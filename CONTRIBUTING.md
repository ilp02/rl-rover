# Contributing

Thanks for helping! This project is educational and experimental.

## How to run quickly
- PC: `python controls\pc_client\run_tracking_server.py`
- Pi: `python controls\rovers\rover_client.py`

## Dev workflow
1. Open an issue first for larger changes.
2. Fork → branch → PR. Keep PRs small and focused.
3. Add a short test or demo log if you change networking/control.
4. Use Black/ruff (PC) and `flake8` (Pi) to lint.

## Good first issues
- Document camera calibration
- Try training models with increasing noise levels
- Search for better alternatives than ArUco markers
- Improve rover hardware