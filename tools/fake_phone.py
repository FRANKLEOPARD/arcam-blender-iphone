"""Simulates the iPhone so you can test Blender without a phone.

Usage:
    python fake_phone.py [ip] [port] [--noise MM] [--record SECONDS]

    ip, port        where Blender listens (defaults 127.0.0.1 9000)
    --noise MM      add Gaussian jitter to the position (millimetres) and ~0.05 deg to the rotation,
                    to try the live smoothing
    --record N      after 1 s send "start recording", after N more seconds "stop recording", then exit
                    (tests the phone -> Blender commands and Blender -> phone status)

The camera walks in a 2 m circle and looks around, 60 packets/s. Ctrl+C to stop.
"""
import argparse
import math
import random
import socket
import time

ap = argparse.ArgumentParser()
ap.add_argument("ip", nargs="?", default="127.0.0.1")
ap.add_argument("port", nargs="?", type=int, default=9000)
ap.add_argument("--noise", type=float, default=0.0)
ap.add_argument("--record", type=float, default=0.0)
args = ap.parse_args()

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(0.0)           # non-blocking: replies from Blender are polled
dest = (args.ip, args.port)
t0 = time.time()
seq = 0
sent_start = sent_stop = False
last_st = None
try:
    while True:
        t = time.time() - t0
        a = t * 0.5                                   # ARKit coords: Y up, -Z forward
        n = args.noise / 1000.0
        px = 2 * math.sin(a) + random.gauss(0, n)
        py = 1.5 + random.gauss(0, n)
        pz = -2 + 2 * math.cos(a) + random.gauss(0, n)
        yaw = -a + random.gauss(0, math.radians(0.05) if args.noise else 0.0)
        qx, qy, qz, qw = 0.0, math.sin(yaw / 2), 0.0, math.cos(yaw / 2)
        seq += 1
        s.sendto(f"AR1,{seq},{time.time():.6f},{px:.5f},{py:.5f},{pz:.5f},"
                 f"{qx:.7f},{qy:.7f},{qz:.7f},{qw:.7f},2".encode(), dest)

        if args.record > 0:
            if not sent_start and t > 1.0:
                s.sendto(b"CMD,rec_start", dest)
                sent_start = True
                print("-> CMD,rec_start")
            elif sent_start and not sent_stop and t > 1.0 + args.record:
                s.sendto(b"CMD,rec_stop", dest)
                sent_stop = True
                print("-> CMD,rec_stop")
            elif sent_stop and t > 2.0 + args.record:
                break

        try:                                           # status from Blender
            while True:
                msg = s.recv(256).decode("ascii", "ignore")
                if msg.startswith("ST,") and msg != last_st:
                    last_st = msg
                    print("<-", msg, " (recording, samples, seconds, takes saved)")
        except OSError:                                # nothing to read yet, or Windows UDP reset errors
            pass
        time.sleep(1 / 60)
except KeyboardInterrupt:
    pass
