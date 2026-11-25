"""Simulates the iPhone so you can test Blender without a phone.
Usage:  python fake_phone.py [ip] [port]      (defaults 127.0.0.1 9000)
The camera walks in a 2 m circle and looks around, 60 packets/s. Ctrl+C to stop."""
import math, socket, sys, time

ip = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
port = int(sys.argv[2]) if len(sys.argv) > 2 else 9000
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
t0 = time.time(); seq = 0
try:
    while True:
        t = time.time() - t0
        a = t * 0.5                                   # ARKit coords: Y up, -Z forward
        px, py, pz = 2 * math.sin(a), 1.5, -2 + 2 * math.cos(a)
        yaw = -a                                      # rotation about Y
        qx, qy, qz, qw = 0.0, math.sin(yaw / 2), 0.0, math.cos(yaw / 2)
        seq += 1
        s.sendto(f"AR1,{seq},{time.time():.6f},{px:.5f},{py:.5f},{pz:.5f},"
                 f"{qx:.7f},{qy:.7f},{qz:.7f},{qw:.7f},2".encode(), (ip, port))
        time.sleep(1 / 60)
except KeyboardInterrupt:
    pass
