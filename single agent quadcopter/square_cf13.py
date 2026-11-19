"""
Fly Crazyflie 2.1 cf13 around the square path from basic_swarm.py.

Same structure as basic_swarm.py (Swarm + hyunlabutils MocapThread), cut down
to one quadcopter, with pre-flight checks that say exactly which stage failed.

    python square_cf13.py --check-only   # run the checks, never arm or fly
    python square_cf13.py                # run the checks, then fly the square

Press Ctrl+C during the flight to cut the motors.
"""
import argparse
import socket
import sys
import threading
import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.reset_estimator import reset_estimator

mocap_system_type = 'vicon'
host_name = '192.168.1.115'
VICON_PORT = 801

# Radio URI and Vicon rigid body are set independently. In basic_swarm.py each
# quadcopter has its own address: cf13 -> ...E713 (URI1), cf14 -> ...E714 (URI2).
# This quadcopter is tracked as cf13 but answers on the radio at ...E709.
URI = 'radio://0/80/2M/E7E7E7E709'
RIGID_BODY = 'cf13'

ADDRESS = int(URI.rsplit('/', 1)[1], 16)
CF_NUMBER = RIGID_BODY

SIDE = 1.5        # square side length [m]
LEG_TIME = 5.0    # seconds per side
TAKEOFF_Z = 2.0   # [m]


class CheckFailed(Exception):
    pass


def stage(n, title):
    print(f'\n[{n}] {title}')


def check_radio():
    stage(1, 'Crazyradio USB dongle')
    try:
        cflib.crtp.init_drivers()
        from cflib.drivers.crazyradio import Crazyradio
        radio = Crazyradio(devid=0)
        print(f'    OK   Crazyradio found, firmware version {radio.version}')
        radio.close()
    except Exception as e:
        raise CheckFailed(
            f'Could not open the Crazyradio: {type(e).__name__}: {e}\n'
            '    -> Dongle unplugged, in use by another program (cfclient?), or the\n'
            '       Windows USB driver is one cflib cannot use (needs WinUSB/libusbK).')


def check_scan():
    stage(2, f'Radio scan for {CF_NUMBER} (address {ADDRESS:010X})')
    found = [f[0] for f in cflib.crtp.scan_interfaces(ADDRESS)]
    if URI in found:
        print(f'    OK   {URI} answered')
        return
    if found:
        raise CheckFailed(
            f'Address {ADDRESS:010X} answered, but not on channel 80 / 2M.\n'
            f'    Found instead: {found}\n'
            '    -> Change URI in this script or the radio settings on the Crazyflie.')
    default = [f[0] for f in cflib.crtp.scan_interfaces(0xE7E7E7E7E7)]
    hint = (f'    A Crazyflie on the factory address answered: {default}\n'
            f'    -> This one may not have been given address {ADDRESS:010X} yet.'
            if default else
            '    Nothing answered on the factory address E7E7E7E7E7 either.\n'
            '    -> Is the Crazyflie powered on (blue LEDs lit, M1 red LED blinking)\n'
            '       and within range of the dongle?')
    raise CheckFailed(f'No Crazyflie answered at {ADDRESS:010X} on any channel/datarate.\n{hint}')


def check_mocap():
    stage(3, f'Motion capture ({mocap_system_type} at {host_name}, rigid body "{RIGID_BODY}")')
    try:
        socket.create_connection((host_name, VICON_PORT), timeout=3).close()
    except OSError as e:
        raise CheckFailed(
            f'Cannot reach {host_name}:{VICON_PORT}: {e}\n'
            '    -> This computer must be on the lab network (e.g. HyunLab-B022C WiFi)\n'
            '       and Vicon Tracker must be running on the host.')
    print(f'    OK   {host_name}:{VICON_PORT} is reachable')

    import motioncapture
    result = {}

    def probe():
        try:
            mc = motioncapture.connect(mocap_system_type, {'hostname': host_name})
            names = set()
            for _ in range(50):
                mc.waitForNextFrame()
                names.update(mc.rigidBodies.keys())
                if RIGID_BODY in names:
                    break
            result['names'] = names
        except Exception as e:
            result['error'] = e

    t = threading.Thread(target=probe, daemon=True)
    t.start()
    t.join(timeout=10)
    if t.is_alive():
        raise CheckFailed('Connected to the mocap host but no frames arrived within 10 s.')
    if 'error' in result:
        raise CheckFailed(f'motioncapture.connect failed: {result["error"]}')
    if RIGID_BODY not in result['names']:
        raise CheckFailed(
            f'Mocap is streaming, but rigid body "{RIGID_BODY}" is not in it.\n'
            f'    Bodies seen: {sorted(result["names"]) or "none"}\n'
            '    -> Enable/create the object in Vicon Tracker and put it in view.')
    print(f'    OK   rigid body "{RIGID_BODY}" is being tracked')


def init(scf: SyncCrazyflie):
    cf = scf.cf

    cf.param.set_value('stabilizer.estimator', '2')
    cf.param.set_value('stabilizer.controller', '2')  # 1: PID, 2: Mellinger, 5: Lee

    reset_estimator(cf)

    cf.platform.send_arming_request(True)
    time.sleep(1)


def flight(scf: SyncCrazyflie):
    hlc = scf.cf.high_level_commander

    hlc.takeoff(TAKEOFF_Z, 2.0)
    print('Taking off...')
    time.sleep(3)

    # Square path from basic_swarm.py: +x, +z, -x, -z (relative moves)
    legs = [
        (-1, -1, 0.0),
        (0.0, 0.0, SIDE),
        (-SIDE, 0.0, 0.0),
        (0.0, 0.0, -SIDE),
    ]
    for i, (dx, dy, dz) in enumerate(legs, start=1):
        print(f'Square side {i}/4: dx={dx:+.1f} dy={dy:+.1f} dz={dz:+.1f}')
        hlc.go_to(dx, dy, dz, 0.0, LEG_TIME, relative=True)
        time.sleep(LEG_TIME)

    hlc.land(0.85, 2.0)
    print('Landing...')
    time.sleep(3)

    hlc.stop()


def fly():
    from hyunlabutils.mocap_thread import MocapThread

    stage(4, f'Connect to {URI}')
    mocap_threads = {URI: [MocapThread(mocap_system_type, host_name, RIGID_BODY)]}
    try:
        with Swarm({URI}, factory=CachedCfFactory(rw_cache='./cache')) as swarm:
            print('    OK   connected')

            # Start sending mocap data to the Crazyflie
            swarm.parallel_safe(lambda scf, mocap_thread: setattr(mocap_thread, 'cf', scf.cf),
                                args_dict=mocap_threads)

            stage(5, 'Fly the square')
            try:
                swarm.parallel_safe(init)
                swarm.parallel_safe(flight)
            except BaseException:
                print('!!! Flight interrupted - stopping motors')
                swarm.parallel(lambda scf: scf.cf.high_level_commander.stop())
                raise
            finally:
                swarm.parallel(lambda scf: scf.cf.platform.send_arming_request(False))
            print('    OK   square complete')
    finally:
        for mocap_thread in mocap_threads.values():
            mocap_thread[0].close()


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--check-only', action='store_true',
                        help='run the pre-flight checks and exit without arming or flying')
    args = parser.parse_args()

    print(f'Quadcopter {CF_NUMBER}: {URI}, mocap body "{RIGID_BODY}"')
    sys.stdout.flush()

    failures = []
    for check in (check_radio, check_scan, check_mocap):
        try:
            check()
        except CheckFailed as e:
            print(f'    FAIL {e}')
            failures.append(check.__name__)
            if check is check_radio:
                print('\n[2] Radio scan skipped - no usable Crazyradio.')
                failures.append('check_scan (skipped)')
                try:
                    check_mocap()
                except CheckFailed as e2:
                    print(f'    FAIL {e2}')
                    failures.append('check_mocap')
                break

    if failures:
        print(f'\nNOT FLYING. Failed: {", ".join(failures)}')
        return 1
    if args.check_only:
        print('\nAll pre-flight checks passed (--check-only, not flying).')
        return 0

    fly()
    return 0


if __name__ == '__main__':
    sys.exit(main())
