# Contributing

Thanks for helping! This project is educational and experimental.

## How to run quickly
- PC: `python control/pc_client/run_tracking_server.py`
- Pi: `python control/pi_agent/run_agent.py --server <PC_IP>:<PORT> --id <4 or 5>`

## Dev workflow
1. Open an issue first for larger changes.
2. Fork → branch → PR. Keep PRs small and focused.
3. Add a short test or demo log if you change networking/control.
4. Use Black/ruff (PC) and `flake8` (Pi) to lint.

## Good first issues
- Document camera calibration (`playground/CALIBRATION.md`)
- Add timestamps to pose packets; sync on Pi side
- Replace SW PWM with `pigpio` and measure jitter

## Code of Conduct
See `CODE_OF_CONDUCT.md` (Contributor Covenant 2.1).
