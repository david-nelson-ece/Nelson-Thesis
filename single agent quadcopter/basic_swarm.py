import socket
import sys
import threading
import time
import numpy as np

from hyunlabutils.mocap_thread import MocapThread

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.reset_estimator import reset_estimator

from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

mocap_system_type = 'vicon'
host_name = '192.168.1.115'
VICON_PORT = 801

# Connectivity checks taken from square_cf13.py. One quadcopter: cf14, which
# answers on the radio at ...E714. (The earlier quadcopter answers at ...E709.)
URI1 = 'radio://0/80/2M/E7E7E7E714'
RIGID_BODY1 = 'cf14'

uris = {
    URI1,
}

ADDRESS1 = int(URI1.rsplit('/', 1)[1], 16)

cflib.crtp.init_drivers()


class CheckFailed(Exception):
    pass


def check_radio():
    print('\n[1] Crazyradio USB dongle')
    try:
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
    print(f'\n[2] Radio scan for {RIGID_BODY1} (address {ADDRESS1:010X})')
    found = [f[0] for f in cflib.crtp.scan_interfaces(ADDRESS1)]
    if URI1 in found:
        print(f'    OK   {URI1} answered')
        return
    if found:
        raise CheckFailed(
            f'Address {ADDRESS1:010X} answered, but not on channel 80 / 2M.\n'
            f'    Found instead: {found}')
    raise CheckFailed(
        f'No Crazyflie answered at {ADDRESS1:010X} on any channel/datarate.\n'
        '    -> Is the Crazyflie powered on and within range of the dongle?')


def check_mocap():
    print(f'\n[3] Motion capture ({mocap_system_type} at {host_name}, rigid body "{RIGID_BODY1}")')
    try:
        socket.create_connection((host_name, VICON_PORT), timeout=3).close()
    except OSError as e:
        raise CheckFailed(
            f'Cannot reach {host_name}:{VICON_PORT}: {e}\n'
            '    -> This computer must be on the lab network (HyunLab-B022C WiFi)\n'
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
                if RIGID_BODY1 in names:
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
    if RIGID_BODY1 not in result['names']:
        raise CheckFailed(
            f'Mocap is streaming, but rigid body "{RIGID_BODY1}" is not in it.\n'
            f'    Bodies seen: {sorted(result["names"]) or "none"}\n'
            '    -> Enable/create the object in Vicon Tracker and put it in view.')
    print(f'    OK   rigid body "{RIGID_BODY1}" is being tracked')

def init(scf: SyncCrazyflie):
    cf = scf.cf

    # Set parameters    
    cf.param.set_value('stabilizer.estimator', '2')
    cf.param.set_value('stabilizer.controller', '2') # 1: PID, 2: Mellinger, 5: Lee

    reset_estimator(cf)

    # Arm the Crazyflie
    cf.platform.send_arming_request(True)
    time.sleep(1)

def flight(scf: SyncCrazyflie):
    cf = scf.cf

    # Take off
    cf.high_level_commander.takeoff(2.0, 2.0)
    print('Taking off...')
    time.sleep(10)

    cf.high_level_commander.go_to(1.0, -1, 0, 0.0, 5.0, relative=True)
    time.sleep(5)

    cf.high_level_commander.go_to(-1.0, 1, 0, 0.0, 3.0, relative=True)
    time.sleep(5)

    cf.high_level_commander.go_to(1.0, 1.0, 0, 0.0, 3.0, relative=True)
    time.sleep(5)

    cf.high_level_commander.go_to(1.0, -1.0, 0, 0.0, 3.0, relative=True)
    time.sleep(5)
    
    cf.high_level_commander.go_to(0, 0, 0, 0.0, 3.0, relative=True)
    time.sleep(5)

    # Land
    cf.high_level_commander.land(0.85, 2.0)
    print('Landing...')
    time.sleep(3)

    # Turn off motors
    cf.high_level_commander.stop()

# Pre-flight checks: stop here, before arming, and say which stage failed.
#   python basic_swarm.py --check-only   runs the checks and exits without flying
failures = []
for check in (check_radio, check_scan, check_mocap):
    try:
        check()
    except CheckFailed as e:
        print(f'    FAIL {e}')
        failures.append(check.__name__)
        if check is check_radio:
            break

if failures:
    print(f'\nNOT FLYING. Failed: {", ".join(failures)}')
    sys.exit(1)
if '--check-only' in sys.argv:
    print('\nAll pre-flight checks passed (--check-only, not flying).')
    sys.exit(0)

print(f'\n[4] Connect to {URI1}')
mocap_threads = {
    URI1: [MocapThread(mocap_system_type, host_name, RIGID_BODY1)],
}

try:
    with Swarm(uris, factory=CachedCfFactory(rw_cache='./cache')) as swarm:
        print('    OK   connected')

        # Start sending mocap data to the Crazyflies
        swarm.parallel_safe(lambda scf, mocap_thread: setattr(mocap_thread, 'cf', scf.cf), args_dict=mocap_threads)

        print('\n[5] Fly')
        try:
            swarm.parallel_safe(init)
            swarm.parallel_safe(flight)
        except BaseException:
            # Ctrl+C or any error during the flight: cut the motors
            print('!!! Flight interrupted - stopping motors')
            swarm.parallel(lambda scf: scf.cf.high_level_commander.stop())
            raise
        finally:
            swarm.parallel(lambda scf: scf.cf.platform.send_arming_request(False))
finally:
    for mocap_thread in mocap_threads.values():
        mocap_thread[0].close()