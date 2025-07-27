# AI Rover Tag (Open)

**Status:** experimental / learning project. Two differential-drive rovers play tag in a 1.5 m × 1 m arena. Overhead camera tracks ArUco markers; a PC sends poses to each rover over sockets; each Pi Zero 2 W runs a small client that converts poses to wheel commands via a learned policy.

## Demo goal
Reproduce a short tag game: two rovers (IDs 4 & 5) in a field marked by ArUco IDs 0–3, controlled by policies trained in Unity ML‑Agents.

## Hardware (quick)
- Chassis: Adafruit aluminum kit (2× DC motors)
- Motor driver: DRV8833
- Compute: Raspberry Pi Zero 2 W (×2)
- Power: Li‑ion pack for Pi; **6 V** AA pack for motors
- Overhead: PC + webcam (top-down view)

Details: `hardware/BOM.md` and `hardware/WIRING.md`.

## Software (quick)
- Unity `<FILL ME: version>` + ML‑Agents `<FILL ME: version>`
- Python `<FILL ME>` on PC/Pi
- OpenCV (ArUco), simple Kalman filter
- TCP Sockets

## How it fits
[Webcam+PC] --poses--> [Pi Rover] --PWM--> [DRV8833] --motors-->
--poses--> [Pi Rover]
Arena defined by ArUco IDs 0–3; rovers carry IDs 4–5.


## Quick start (PC)
1. Install deps: `pip install -r control/pc_client/requirements.txt`
2. Print and place `playground/markers/` (IDs 0–3 corners, 4–5 on rovers).
3. Run tracking server: `python control/pc_client/run_tracking_server.py`
4. Start rover clients (on each Pi): see `control/pi_agent/README.md`
5. Watch terminal output; press `Y` to emergency stop.

## Training (ML‑Agents)
- Configs: `ml/configs/`
- Notes & reward design: `ml/README.md`
- Export/inference: `ONNX`

## Known limitations
- SW PWM on Pi can jitter under load; consider hardware PWM.
- Domain randomization is minimal; outdoors transfer not robust.

## Roadmap
- [ ] Replace SW PWM with hardware PWM or `pigpio`
- [ ] Camera calibration & homography guide
- [ ] Improve reward shaping for capture/escape
- [ ] Packet timestamps + latency compensation

## License
- Code: `<FILL ME: MIT/Apache-2.0>`
- Docs/Data: `<FILL ME: CC BY 4.0/CC0>`
- (If hardware files are added) Hardware: `<FILL ME: CERN OHL-S/W/P>`
