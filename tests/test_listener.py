"""Tests for the network half of the add-on. bpy / mathutils are stubbed, so this runs anywhere."""
import os, socket, subprocess, sys, threading, time, types
from unittest.mock import MagicMock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

bpy = types.ModuleType("bpy")
class _T: pass
bpy.types = types.SimpleNamespace(PropertyGroup=_T, Operator=_T, Panel=_T, Object=_T, Scene=_T)
bpy.props = types.ModuleType("bpy.props")
for n in "BoolProperty FloatProperty IntProperty PointerProperty StringProperty".split():
    setattr(bpy.props, n, lambda *a, **k: None)
sys.modules.update({"bpy": bpy, "bpy.props": bpy.props, "mathutils": MagicMock()})
sys.path.insert(0, os.path.join(ROOT, "blender"))
import arcam_blender_addon as A


def run_listener(sock):
    ev = threading.Event()
    th = threading.Thread(target=A._listen, args=(sock, ev))
    th.start()
    return ev, th


def test_parse():
    ok = A._parse(b"AR1,1,100.5,1,2,3,0,0,0,1,2")
    assert ok and ok[3] == (1.0, 0.0, 0.0, 0.0) and ok[4] == 2
    assert A._parse(b"garbage") is None
    assert A._parse(b"AR1,1,x,1,2,3,0,0,0,1,2") is None          # bad number
    assert A._parse(b"AR1,1,1,1,2,3,0,0,0,0,2") is None          # zero quaternion
    assert A._parse(b"AR1,1,1,nan,2,3,0,0,0,1,2") is None        # NaN
    assert A._parse(b"AR1,1,1,1,2,3,0,0,0,1") is None            # too short


def test_out_of_order():
    A.STATE.reset()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0)); s.settimeout(0.2)
    port = s.getsockname()[1]
    ev, th = run_listener(s)
    c = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for i, ts in enumerate([10.0, 10.02, 10.01, 10.03]):
        c.sendto(f"AR1,{i},{ts},0,0,0,0,0,0,1,2".encode(), ("127.0.0.1", port))
    time.sleep(0.5); ev.set(); th.join(); s.close()
    assert A.STATE.counter == 3, A.STATE.counter


def test_connection_reset_survives():
    A.STATE.reset()
    class FakeSock:
        n = 0
        def recvfrom(self, size):
            self.n += 1
            if self.n == 1: raise ConnectionResetError(10054, "reset")
            if self.n == 2: return b"AR1,1,5.0,0,0,0,0,0,0,1,2", ("x", 1)
            time.sleep(0.01); raise socket.timeout()
    ev, th = run_listener(FakeSock())
    time.sleep(0.3); ev.set(); th.join()
    assert A.STATE.counter == 1


def test_fake_phone():
    A.STATE.reset()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0)); s.settimeout(0.2)
    ev, th = run_listener(s)
    p = subprocess.Popen([sys.executable, os.path.join(ROOT, "tools", "fake_phone.py"),
                          "127.0.0.1", str(s.getsockname()[1])])
    time.sleep(1.0); p.terminate(); p.wait(); ev.set(); th.join(); s.close()
    assert A.STATE.counter > 20, A.STATE.counter


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print("ok", name)
