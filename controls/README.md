# Scripts for PC and Rovers

A file list for running the rover system.

## Rovers (`./rovers`)

| File | Purpose | How to run |
| --- | --- | --- |
| `motor_test.py` | Simple motor script for testing | On the rover: `python motor_test.py` |
| `rover_client.py` | Main rover client. Connects to the PC, receives pose, and drives motors | SSH into the rover, then run: `python rover_client.py`. Edit the script for local IP address. |

## PC Clients (`./pc_client`)

| File | Purpose | How to run |
| --- | --- | --- |
| `run_tracking_server.py` | Main PC script. Runs ArUco detection, tracking, and sends poses to rovers | On the PC: `python run_tracking_server.py` after both rover scripts are running |
