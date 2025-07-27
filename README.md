# AI Rover Tag (Open)

**Status:** experimental / learning project. Two differential-drive rovers play tag in a 1.5 m × 1 m arena. Overhead camera tracks ArUco markers; a PC sends poses to each rover over sockets; each Pi Zero 2 W runs a small client that converts poses to wheel commands via a learned policy.

## Demo goal
Reproduce a 60‑second tag game: two rovers (IDs 4 & 5) in a field marked by ArUco IDs 0–3, controlled by policies trained in Unity ML‑Agents.

## Hardware (quick)
- Chassis: Adafruit aluminum kit (2× DC motors)
- Motor driver: DRV8833
- Compute: Raspberry Pi Zero 2 W (×2)
- Power: Li‑ion pack for Pi; **6 V** AA pack for motors
- Overhead: PC + webcam (bird’s‑eye)

Details: `hardware/BOM.md` and `hardware/WIRING.md`.

## Software (quick)
- Unity `<FILL ME: version>` + ML‑Agents `<FILL ME: version>`
- Python `<FILL ME>` on PC/Pi
- OpenCV (ArUco), simple Kalman filter
- Sockets: `<FILL ME: TCP/UDP>`

## How it fits
