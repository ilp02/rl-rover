#!/usr/bin/env python3
import socket, struct, threading, time
import numpy as np
import onnxruntime as ort
import RPi.GPIO as GPIO

HOST, PORT = "0.0.0.0", 9991
PKT_IN  = struct.Struct("!9d")
PKT_OUT = struct.Struct("!3d")

# ── DRV8833 (BCM pins) ─────────────────────────────────────
PIN_L_FWD, PIN_L_REV = 27, 17
PIN_R_FWD, PIN_R_REV = 22, 23
PWM_HZ = 200
DUTY_MIN, DUTY_MAX = 20, 50   # % duty window

GPIO.setmode(GPIO.BCM)
_channels = []
for pin in (PIN_L_FWD, PIN_L_REV, PIN_R_FWD, PIN_R_REV):
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, PWM_HZ); pwm.start(0)
    _channels.append(pwm)

def set_wheel(idx: int, cmd: float):
    pwm_fwd, pwm_rev = (_channels[0], _channels[1]) if idx == 0 else (_channels[2], _channels[3])
    if abs(cmd) < 0:
        pwm_fwd.ChangeDutyCycle(0); pwm_rev.ChangeDutyCycle(0)
        print(f"W{idx}: 0%, {cmd}")
        return

    duty = DUTY_MIN + (DUTY_MAX - DUTY_MIN) * min(1.0, abs(cmd))
    print(f"W{idx}: {duty}%, {cmd}")
    if cmd > 0:
        pwm_fwd.ChangeDutyCycle(duty); pwm_rev.ChangeDutyCycle(0)
    else:
        pwm_rev.ChangeDutyCycle(duty); pwm_fwd.ChangeDutyCycle(0)

def stop_motors():
    for pwm in _channels: pwm.ChangeDutyCycle(0)

# ── ONNX ───────────────────────────────────────────────────
ORT_SESSION = ort.InferenceSession("/home/pi/codes/py311/codes/models/Jerry.onnx",
                                   providers=["CPUExecutionProvider"])
IN_NAME  = ORT_SESSION.get_inputs()[0].name           # 'obs_0'
OUT_NAME = ORT_SESSION.get_outputs()[4].name          # deterministic_continuous_actions

def recvall(sock: socket.socket, n: int) -> bytes:
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk: raise ConnectionError("peer closed")
        buf += chunk
    return buf

# ── shared latest pose + event ─────────────────────────────
latest_pose = None
pose_lock   = threading.Lock()
pose_ready  = threading.Event()

# ── 10 Hz drive loop ───────────────────────────────────────
RATE_HZ = 10.0
PERIOD  = 1.0 / RATE_HZ

def drive_loop(stop_evt: threading.Event):
    # Wait until we have the first pose
    while not stop_evt.is_set() and not pose_ready.wait(timeout=0.1):
        pass

    last_pose = None
    t_next = time.perf_counter()

    while not stop_evt.is_set():
        # sleep until next 10 Hz tick
        now = time.perf_counter()
        if now < t_next:
            time.sleep(t_next - now)
        t_next += PERIOD
        # avoid drift if we ever fall far behind
        if t_next - now > 5*PERIOD:
            t_next = time.perf_counter() + PERIOD

        # read the most recent pose (reuse previous if none new)
        with pose_lock:
            p = latest_pose.copy() if latest_pose is not None else None
        if p is not None:
            last_pose = p
        if last_pose is None:
            continue

        # inference @ 10 Hz
        inp = last_pose.astype(np.float32, copy=False).reshape(1, -1)
        left, right = ORT_SESSION.run([OUT_NAME], {IN_NAME: inp})[0][0]

        set_wheel(0, float(left))
        set_wheel(1, float(right))

    stop_motors()

# ── network handler (no rate limit) ────────────────────────
def client_thread(conn: socket.socket, stop_evt: threading.Event):
    try:
        while True:
            raw  = recvall(conn, PKT_IN.size)
            pose = np.asarray(PKT_IN.unpack(raw), dtype=np.float32)
            with pose_lock:
                global latest_pose
                latest_pose = pose
                pose_ready.set()
            conn.sendall(PKT_OUT.pack(0.0, 0.0, float(pose[-1])))
    except ConnectionError:
        print("[NET] disconnected")
    finally:
        conn.close()
        stop_evt.set()

def main():
    stop_evt = threading.Event()
    threading.Thread(target=drive_loop, args=(stop_evt,), daemon=True).start()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((HOST, PORT)); srv.listen(1)
        print(f"DRV8833 rover ready on :{PORT}")
        try:
            while not stop_evt.is_set():
                conn, _ = srv.accept()
                print("[NET] PC connected")
                threading.Thread(target=client_thread, args=(conn, stop_evt), daemon=True).start()
        except KeyboardInterrupt:
            print("Ctrl-C — exit"); stop_evt.set()

    stop_motors(); GPIO.cleanup()

if __name__ == "__main__":
    main()
