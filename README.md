# Nelson Thesis

## single agent quadcopter

Single Crazyflie 2.1 flights in the Hyun Research Lab, positioned by Vicon
motion capture.

- `basic_swarm.py`: main flight script (currently `cf14`, radio address `E7E7E7E714`).
  Runs pre-flight checks (Crazyradio, radio scan, Vicon object) before arming.
- `square_cf13.py`: earlier square-path script for `cf13`.
- `flight_logs/`: Vicon tracks recorded during test flights.

### Setup

Python 3.13, a Crazyradio, and this computer on the lab network (Vicon host `192.168.1.115`).

```bash
python -m venv .venv
.venv\Scripts\python -m pip install -r "single agent quadcopter/requirements.txt"
```

### Run

```bash
python basic_swarm.py --check-only   # checks only, never arms
python basic_swarm.py                # checks, then flies
```
