# AI Rover Tag (Open)

**Status:** experimental / learning project. Two differential-drive rovers play tag in a 1.5 m × 1 m arena. Overhead camera tracks ArUco markers; a PC sends poses to each rover over sockets; each Pi Zero 2 W runs a small client that converts poses to wheel commands via a learned policy.

## Demo goal
Reproduce a short tag game: two rovers in a field marked by ArUco markers, controlled by policies trained in Unity ML‑Agents.

## Hardware (quick)
- Chassis: Adafruit aluminum kit (2× DC motors)
- Motor driver: DRV8833
- Compute: Raspberry Pi Zero 2 W (×2)
- Power: Li‑ion pack for Pi; 3 V AA pack for motors
- Positioning: PC + webcam (top-down view)

Details: `hardware/BOM.md` and `hardware/INSTRUCTIONS.md`.

## Software (quick)
- Unity `2022.3.16f1` + ML‑Agents
- Python `3.12` on PC, Python `3.10` on Pi
- OpenCV (ArUco), simple Kalman filter
- TCP Socket over local network

## Quick start (PC)
1. Install deps: `pip install -r controls/rovers/requirements.txt`
2. Print and place ArUco markers (IDs 0–3 corners, 4–5 on rovers).
3. Run tracking server: `python control/pc_client/run_tracking_server.py`
4. Start rover clients (on each Pi).
5. Watch terminal output; press `Y` to emergency stop.

## Training (ML‑Agents)
- Configs: `ml\training\tag_config.yaml`
- Notes & reward design: `ml\README.md`
- Export/inference: `ONNX`

## Known limitations
- 3V battery pack is not enough voltage for motors, recommend switching to 6V.
- Domain randomization is minimal; transfer does not work.

## Roadmap
- [ ] Increased domain randomization to bridge the 'Reality gap'.
- [ ] Camera calibration & homography guide
- [ ] Replace SW PWM with hardware PWM
- [ ] Improve reward shaping for capture/escape