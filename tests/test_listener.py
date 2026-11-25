"""Tests for the add-on logic. bpy is stubbed and mathutils is replaced by a numpy stand-in
(tests/fake_mathutils.py), so everything runs outside Blender:  python tests/test_listener.py"""
import contextlib
import io
import math
import os
import random
import socket
import subprocess
import sys
import tempfile
import threading
import time
import types
from types import SimpleNamespace as NS

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "blender"))
import fake_mathutils as FM  # noqa: E402

# Keep every file the add-on writes (emergency saves go to ~/ARCam_takes) inside a temp folder.
FAKE_HOME = tempfile.mkdtemp()
os.environ["HOME"] = FAKE_HOME
os.environ["USERPROFILE"] = FAKE_HOME

# ------------------------------------------------------------------ minimal bpy stand-in
bpy = types.ModuleType("bpy")


class _T:
    pass


bpy.types = NS(PropertyGroup=_T, Operator=_T, Panel=_T, Object=_T, Scene=_T)
bpy.props = types.ModuleType("bpy.props")
for _n in "BoolProperty FloatProperty IntProperty PointerProperty StringProperty".split():
    setattr(bpy.props, _n, lambda *a, **k: None)
bpy.path = NS(abspath=lambda d: d)
SCENE_BY_NAME = {}
bpy.data = NS(scenes=NS(get=lambda name: SCENE_BY_NAME.get(name)), filepath="")
bpy.context = NS(window_manager=NS(windows=[]), scene=None)
_TIMERS = set()
bpy.app = NS(
    timers=NS(register=lambda f, **k: _TIMERS.add(f), unregister=lambda f: _TIMERS.discard(f),
              is_registered=lambda f: f in _TIMERS),
    handlers=NS(load_pre=[], persistent=lambda f: f),
)
sys.modules.update({"bpy": bpy, "bpy.props": bpy.props, "mathutils": FM})
import arcam_blender_addon as A  # noqa: E402


# ------------------------------------------------------------------ helpers
def run_listener(sock):
    ev = threading.Event()
    th = threading.Thread(target=A._listen, args=(sock, ev))
    th.start()
    return ev, th


def udp_pair():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    s.settimeout(0.2)
    c = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return s, c, ("127.0.0.1", s.getsockname()[1])


class FakeCam:
    """Mimics what the add-on uses of a Blender camera object, including the fact that
    keyframe_insert() creates a new action when none is assigned."""

    def __init__(self, matrix=None, parent=None):
        self.matrix_world = matrix or FM.Matrix()
        self.matrix_parent_inverse = FM.Matrix()
        self.parent = parent
        self.location = (0, 0, 0)
        self.rotation_euler = None
        self.rotation_mode = "XYZ"
        self.keys = []          # (prop, frame, value, action_name)
        self.animation_data = None
        self.type = "CAMERA"

    def animation_data_create(self):
        if self.animation_data is None:
            self.animation_data = NS(action=None)
        return self.animation_data

    def keyframe_insert(self, prop, frame):
        ad = self.animation_data_create()
        if ad.action is None:
            ad.action = NS(name="CameraAction", use_fake_user=False)
        e = self.rotation_euler
        value = tuple(self.location) if prop == "location" else (e.x, e.y, e.z)
        self.keys.append((prop, frame, value, ad.action))


def make_scene(cam, **over):
    props = dict(camera=cam, pos_scale=1.0, lock_pos=False, smoothing=0.0, roll_offset=0.0,
                 take_smooth=0.0, round_frames=True, output_dir=tempfile.mkdtemp(), port=9000)
    props.update(over)
    scn = NS(name="Scene", arcam=NS(**props), render=NS(fps=24, fps_base=1.0), frame_current=1,
             frame_end=250, camera=None)
    scn.frame_set = lambda f: setattr(scn, "frame_current", f)
    SCENE_BY_NAME.clear()
    SCENE_BY_NAME["Scene"] = scn
    return scn


def fresh_state(scn, running=True):
    A.STATE.reset()
    A.STATE.scene_name = scn.name
    A.STATE.base = A._cam_base(scn.arcam.camera)
    A.STATE.running = running


def pkt(ts, pos=(0, 0, 0), q=(1, 0, 0, 0), trk=2):
    return (ts, pos, q, trk)


def cam_pos(cam):
    return [round(v, 6) for v in cam.matrix_world.translation]


def base_looking_forward():
    """Blender camera at the origin, rotated 90 deg about X: horizontal, looking along +Y."""
    return FM.Matrix.Rotation(math.radians(90), 4, "X")


def _std(vals):
    m = sum(vals) / len(vals)
    return math.sqrt(sum((v - m) ** 2 for v in vals) / len(vals))


# ------------------------------------------------------------------ protocol / network
def test_parse():
    ok = A._parse(b"AR1,1,100.5,1,2,3,0,0,0,1,2")
    assert ok and ok[3] == (1.0, 0.0, 0.0, 0.0) and ok[4] == 2
    assert A._parse(b"garbage") is None
    assert A._parse(b"AR1,1,x,1,2,3,0,0,0,1,2") is None
    assert A._parse(b"AR1,1,1,1,2,3,0,0,0,0,2") is None          # zero quaternion
    assert A._parse(b"AR1,1,1,nan,2,3,0,0,0,1,2") is None        # NaN
    assert A._parse(b"AR1,1,1,inf,2,3,0,0,0,1,2") is None        # infinity
    assert A._parse(b"AR1,1,1,1,2,3,0,0,0,1") is None            # too short
    assert A._parse(b"AR1,1,1,1,2,3,0,0,0,1,7") is None          # unknown tracking value


def test_parse_cmd():
    assert A._parse_cmd(b"CMD,recenter") == "recenter"
    assert A._parse_cmd(b"CMD,rec_start\n") == "rec_start"
    assert A._parse_cmd(b"CMD,rec_stop") == "rec_stop"
    assert A._parse_cmd(b"CMD,format_disk") is None
    assert A._parse_cmd(b"CMD") is None
    assert A._parse_cmd(b"AR1,1,2,3") is None
    assert A._parse_cmd(b"\xff\xfe") is None


def test_out_of_order():
    A.STATE.reset()
    s, c, dest = udp_pair()
    ev, th = run_listener(s)
    for i, ts in enumerate([10.0, 10.02, 10.01, 10.03]):
        c.sendto(f"AR1,{i},{ts},0,0,0,0,0,0,1,2".encode(), dest)
    time.sleep(0.5); ev.set(); th.join(); s.close()
    assert A.STATE.counter == 3, A.STATE.counter
    assert [p[0] for p in A.STATE.queue] == [10.0, 10.02, 10.03]


def test_phone_restart_accepted():
    A.STATE.reset()
    s, c, dest = udp_pair()
    ev, th = run_listener(s)
    for ts in [1000.0, 1000.02, 5.0, 5.02]:            # clock jumps back by a lot = phone rebooted
        c.sendto(f"AR1,0,{ts},0,0,0,0,0,0,1,2".encode(), dest)
    time.sleep(0.5); ev.set(); th.join(); s.close()
    assert A.STATE.counter == 4


def test_connection_reset_survives():
    A.STATE.reset()

    class FakeSock:
        n = 0

        def recvfrom(self, size):
            self.n += 1
            if self.n == 1:
                raise ConnectionResetError(10054, "reset")
            if self.n == 2:
                return b"AR1,1,5.0,0,0,0,0,0,0,1,2", ("x", 1)
            time.sleep(0.01)
            raise socket.timeout()

    ev, th = run_listener(FakeSock())
    time.sleep(0.3); ev.set(); th.join()
    assert A.STATE.counter == 1


def test_commands_over_udp():
    A.STATE.reset()
    s, c, dest = udp_pair()
    ev, th = run_listener(s)
    c.sendto(b"CMD,recenter", dest)
    c.sendto(b"CMD,nonsense", dest)
    c.sendto(b"CMD,rec_start", dest)
    time.sleep(0.4); ev.set(); th.join(); s.close()
    assert list(A.STATE.cmds) == ["recenter", "rec_start"], list(A.STATE.cmds)
    assert A.STATE.peer is not None and A.STATE.peer[0] == "127.0.0.1"


def test_fake_phone():
    A.STATE.reset()
    s, c, dest = udp_pair()
    ev, th = run_listener(s)
    p = subprocess.Popen([sys.executable, os.path.join(ROOT, "tools", "fake_phone.py"),
                          "127.0.0.1", str(dest[1])])
    time.sleep(1.0); p.terminate(); p.wait(); ev.set(); th.join(); s.close()
    assert A.STATE.counter > 20, A.STATE.counter


def test_local_ips_fast():
    t0 = time.time()
    ips = A._local_ips()
    assert time.time() - t0 < 1.0                     # never freezes Blender (macOS hostname lookups can hang)
    assert isinstance(ips, list) and all(isinstance(i, str) for i in ips)
    assert not any(i.startswith(("127.", "169.254.")) for i in ips)
    assert len(ips) == len(set(ips))


# ------------------------------------------------------------------ math
def test_fake_mathutils_selfcheck():
    rnd = random.Random(1)
    for _ in range(50):
        q = FM.Quaternion([rnd.uniform(-1, 1) for _ in range(4)])
        m = q.to_matrix()
        q2 = m.decompose()[1]
        assert abs(m.m - q2.to_matrix().m).max() < 1e-9
        e = q.to_euler("XYZ")
        r = FM.Matrix.Rotation(e.z, 4, "Z") @ FM.Matrix.Rotation(e.y, 4, "Y") @ FM.Matrix.Rotation(e.x, 4, "X")
        assert abs(m.m - r.m).max() < 1e-9


def test_qslerp():
    a = (1.0, 0.0, 0.0, 0.0)
    b = A._qnorm((math.cos(0.5), 0.0, math.sin(0.5), 0.0))   # 1 rad about Y
    assert max(abs(x - y) for x, y in zip(A._qslerp(a, b, 0.0), a)) < 1e-9
    assert max(abs(x - y) for x, y in zip(A._qslerp(a, b, 1.0), b)) < 1e-9
    assert abs(2 * math.acos(A._qslerp(a, b, 0.5)[0]) - 0.5) < 1e-9
    neg = tuple(-v for v in b)                                # same rotation, opposite sign
    assert abs(2 * math.acos(abs(A._qslerp(a, neg, 0.5)[0])) - 0.5) < 1e-9


def _run_filter(s, fn, seconds=5.0, hz=60.0, t0=100.0):
    f = A.OneEuroPose()
    raw, out = [], []
    for i in range(int(seconds * hz)):
        loc, q = fn(i / hz)
        raw.append(loc)
        out.append(f.update(t0 + i / hz, loc, q, s)[0])
    return raw, out


def test_one_euro_removes_jitter_when_still():
    rnd = random.Random(7)
    q = (1.0, 0.0, 0.0, 0.0)
    noisy = lambda t: ((rnd.gauss(0, 0.002), rnd.gauss(0, 0.002), rnd.gauss(0, 0.002)), q)
    raw, out = _run_filter(0.5, noisy)
    r, o = _std([p[0] for p in raw[60:]]), _std([p[0] for p in out[60:]])
    print(f"   still, s=0.5: jitter {r * 1000:.2f} mm -> {o * 1000:.2f} mm")
    assert o < 0.55 * r
    raw, out = _run_filter(1.0, noisy)
    assert _std([p[0] for p in out[60:]]) < o                 # more smoothing -> less jitter


def test_one_euro_low_lag_when_moving():
    q = (1.0, 0.0, 0.0, 0.0)
    ramp = lambda t: ((1.0 * t, 0.0, 0.0), q)                  # 1 m/s walk
    for s, limit in ((0.0, 0.02), (0.5, 0.05), (1.0, 0.09)):
        raw, out = _run_filter(s, ramp, seconds=4.0)
        lag = raw[-1][0] - out[-1][0]
        print(f"   walking 1 m/s, s={s}: lag {lag * 100:.1f} cm")
        assert 0.0 <= lag < limit, (s, lag)


def test_one_euro_rotation_follows():
    f = A.OneEuroPose()
    q = last = None
    for i in range(240):                                       # 90 deg/s yaw for 4 s
        ang = math.radians(90.0) * i / 60.0
        q = (math.cos(ang / 2), 0.0, math.sin(ang / 2), 0.0)
        last = f.update(10.0 + i / 60.0, (0, 0, 0), q, 0.5)[1]
    err = math.degrees(2 * math.acos(min(1.0, abs(A._qdot(last, q)))))
    print(f"   rotating 90 deg/s, s=0.5: lag {err:.2f} deg")
    assert err < 6.0


def test_one_euro_snaps():
    f = A.OneEuroPose()
    q = (1.0, 0.0, 0.0, 0.0)
    for i in range(10):
        f.update(1.0 + i / 60, (0, 0, 0), q, 0.8)
    assert f.update(1.2, (5.0, 0, 0), q, 0.8)[0] == (5.0, 0, 0)   # 5 m jump = relocalization
    assert f.update(3.0, (6.0, 0, 0), q, 0.8)[0] == (6.0, 0, 0)   # 1.8 s gap
    assert f.update(2.0, (7.0, 0, 0), q, 0.8)[0] == (7.0, 0, 0)   # clock went backwards


def test_smooth_take():
    q = (1.0, 0.0, 0.0, 0.0)
    assert A._smooth_take([(0, (0, 0, 0), q, 2)], 0.05) == [(0, (0, 0, 0), q, 2)]   # too short
    ramp = [(i / 60, (i / 60, 0.0, 0.0), q, 2) for i in range(120)]
    assert A._smooth_take(ramp, 0.0) is ramp                                          # off
    sm = A._smooth_take(ramp, 0.05)
    assert max(abs(sm[i][1][0] - ramp[i][1][0]) for i in range(20, 100)) < 1e-6      # zero lag
    rnd = random.Random(3)
    noisy = [(i / 60, (rnd.gauss(0, 0.002), 0.0, 0.0), q, 2) for i in range(300)]
    a = _std([s[1][0] for s in noisy])
    b = _std([s[1][0] for s in A._smooth_take(noisy, 0.05)])
    print(f"   take smoothing sigma=0.05 s: jitter {a * 1000:.2f} mm -> {b * 1000:.2f} mm")
    assert b < 0.4 * a
    flip = [(i / 60, (0, 0, 0), tuple(v * (-1) ** i for v in q), 2) for i in range(60)]  # q / -q alternating
    assert all(abs(abs(o[2][0]) - 1.0) < 1e-6 for o in A._smooth_take(flip, 0.05))   # no collapse to 0


def test_resample_exact_frames_no_bias():
    q = (1.0, 0.0, 0.0, 0.0)
    take = [(i / 60, (i / 60, 0.0, 0.0), q, 2) for i in range(121)]    # x = t, 2 s at 60 Hz
    keys = A._resample(take, 24.0, 10)
    assert keys[0][0] == 10.0 and keys[-1][0] == 58.0 and len(keys) == 49
    for f, loc, _q in keys:                                             # value at the exact frame time
        assert abs(loc[0] - (f - 10) / 24.0) < 1e-9, (f, loc)
    jittered = [(t + (0.004 if i % 2 else 0.0), l, qq, k) for i, (t, l, qq, k) in enumerate(take)]
    jittered = [(t, (t, 0.0, 0.0), qq, k) for t, l, qq, k in jittered]  # irregular sample times
    for f, loc, _q in A._resample(jittered, 24.0, 1):
        assert abs(loc[0] - (f - 1) / 24.0) < 1e-9
    half = math.radians(45)                                             # rotation interpolates too
    rot = [(0.0, (0, 0, 0), q, 2), (1.0, (0, 0, 0), (math.cos(half), 0, math.sin(half), 0), 2)]
    mid = [k for k in A._resample(rot, 2.0, 0) if k[0] == 1.0][0]       # frame 1 = t 0.5 s
    assert abs(2 * math.acos(mid[2][0]) - math.radians(45)) < 1e-9
    assert A._resample([], 24.0, 1) == []
    one = A._resample([(0.0, (1, 2, 3), q, 2)], 24.0, 5)
    assert one == [(5.0, (1, 2, 3), q)]


# ------------------------------------------------------------------ camera pipeline
def test_pipeline_recenter_and_motion():
    cam = FakeCam(base_looking_forward())
    scn = make_scene(cam)
    fresh_state(scn)
    st, p = A.STATE, scn.arcam
    base = cam.matrix_world.m.copy()
    assert A._process(st, p, cam, [pkt(1.0, pos=(3.0, 1.2, -4.0))])        # first packet defines the start
    assert abs(cam.matrix_world.m - base).max() < 1e-9
    A._process(st, p, cam, [pkt(1.02, pos=(3.0, 1.2, -5.0))])               # phone 1 m forward (-Z)
    assert cam_pos(cam) == [0.0, 1.0, 0.0], cam_pos(cam)                    # camera 1 m along +Y
    A._process(st, p, cam, [pkt(1.04, pos=(4.0, 1.2, -4.0))])               # phone 1 m right (+X)
    assert cam_pos(cam) == [1.0, 0.0, 0.0], cam_pos(cam)
    A._process(st, p, cam, [pkt(1.06, pos=(3.0, 2.2, -4.0))])               # phone 1 m up (+Y)
    assert cam_pos(cam) == [0.0, 0.0, 1.0], cam_pos(cam)                    # camera up (+Z)
    st.recenter = True
    A._process(st, p, cam, [pkt(1.08, pos=(9.0, 9.0, 9.0))])
    assert abs(cam.matrix_world.m - base).max() < 1e-9                      # back to the start pose


def test_pipeline_scale_lock_roll_tracking():
    cam = FakeCam(base_looking_forward())
    scn = make_scene(cam, pos_scale=10.0)
    fresh_state(scn)
    st, p = A.STATE, scn.arcam
    A._process(st, p, cam, [pkt(1.0)])
    A._process(st, p, cam, [pkt(1.02, pos=(0, 0, -1.0))])
    assert cam_pos(cam) == [0.0, 10.0, 0.0]                                 # move scale
    p.pos_scale = 1.0
    p.lock_pos = True
    A._process(st, p, cam, [pkt(1.04, pos=(5, 5, 5))])
    assert cam_pos(cam) == [0.0, 0.0, 0.0]                                  # rotation only
    p.lock_pos = False
    before = cam.matrix_world.m.copy()
    A._process(st, p, cam, [pkt(1.06, pos=(7, 7, 7), trk=0)])               # tracking lost: ignored
    assert abs(cam.matrix_world.m - before).max() < 1e-12


def test_anchor_change_does_not_jump():
    cam = FakeCam(base_looking_forward())
    scn = make_scene(cam)
    fresh_state(scn)
    st, p = A.STATE, scn.arcam
    A._process(st, p, cam, [pkt(1.0)])
    A._process(st, p, cam, [pkt(1.02, pos=(0, 0, -2.0))])                  # walked 2 m
    here = cam.matrix_world.m.copy()
    p.pos_scale = 10.0
    A._on_anchor_change(p, None)                                            # the property's update callback
    A._process(st, p, cam, [pkt(1.04, pos=(0, 0, -2.0))])                  # phone did not move
    assert abs(cam.matrix_world.m - here).max() < 1e-9                      # camera did not jump
    A._process(st, p, cam, [pkt(1.06, pos=(0, 0, -2.1))])                  # 10 cm more ...
    assert abs(cam.matrix_world.translation.y - (here[1, 3] + 1.0)) < 1e-9  # ... = 1 m at scale 10
    st.running = False
    p.pos_scale = 1.0
    A._on_anchor_change(p, None)                                            # not listening: no effect
    assert st.recenter is False


def test_pipeline_rotation_direction():
    cam = FakeCam(base_looking_forward())
    scn = make_scene(cam)
    fresh_state(scn)
    st, p = A.STATE, scn.arcam
    A._process(st, p, cam, [pkt(1.0)])
    yaw = math.radians(30)                                                  # phone turns left
    A._process(st, p, cam, [pkt(1.02, q=(math.cos(yaw / 2), 0, math.sin(yaw / 2), 0))])
    fwd = cam.matrix_world.m[:3, :3] @ [0, 0, -1]
    assert fwd[1] > 0.86 and fwd[0] < -0.49, fwd                            # still forward, turned left


def test_pipeline_records_raw_and_smooths_live():
    cam = FakeCam(base_looking_forward())
    scn = make_scene(cam, smoothing=0.9)
    fresh_state(scn)
    st, p = A.STATE, scn.arcam
    st.recording = True
    rnd = random.Random(5)
    for i in range(120):
        A._process(st, p, cam, [pkt(1.0 + i / 60, pos=(0, 0, -rnd.gauss(0, 0.003)))])
    assert len(st.samples) == 120
    assert _std([s[1][1] for s in st.samples]) > 0.002                     # recorded data is raw
    assert abs(cam.matrix_world.translation.y) < 0.01                       # live camera is calm
    assert st.samples[0][0] == 0.0


def test_pipeline_batches_many_packets_per_tick():
    cam = FakeCam(base_looking_forward())
    scn = make_scene(cam)
    fresh_state(scn)
    st, p = A.STATE, scn.arcam
    st.recording = True
    A._process(st, p, cam, [pkt(1.0 + i / 60, pos=(0, 0, -i / 60)) for i in range(30)])
    assert len(st.samples) == 30                                            # none lost
    assert abs(cam.matrix_world.translation.y - 29 / 60) < 1e-9


# ------------------------------------------------------------------ recording / baking
def record_take(scn, cam, seconds=2.0, hz=60, t0=50.0):
    st, p = A.STATE, scn.arcam
    ok, msg = A._do_rec_start(scn)
    assert ok, msg
    for i in range(int(seconds * hz) + 1):
        A._process(st, p, cam, [pkt(t0 + i / hz, pos=(0.5 * i / hz, 0, -1.0 * i / hz))])
    return A._do_rec_stop(scn)


def test_record_save_and_bake():
    cam = FakeCam(base_looking_forward())
    scn = make_scene(cam)
    fresh_state(scn)
    scn.frame_current = 10
    ok, msg = record_take(scn, cam)
    print("  ", msg)
    assert ok
    files = sorted(os.listdir(scn.arcam.output_dir))
    assert len(files) == 1 and files[0].endswith(".csv")
    rows = open(os.path.join(scn.arcam.output_dir, files[0])).read().strip().splitlines()
    assert rows[0].startswith("time_s,frame,tracking") and len(rows) == 1 + 121
    assert float(rows[1].split(",")[1]) == 10.0
    loc_keys = [(f, v) for prop, f, v, _a in cam.keys if prop == "location"]
    assert [f for f, _v in loc_keys] == [float(f) for f in range(10, 59)]  # 2 s at 24 fps, whole frames
    for f, v in loc_keys:                                                   # exact value at each frame
        t = (f - 10) / 24.0
        assert abs(v[0] - 0.5 * t) < 1e-9 and abs(v[1] - 1.0 * t) < 1e-9, (f, v)
    act = cam.animation_data.action
    assert act.name == os.path.splitext(files[0])[0] and act.use_fake_user   # action named like the CSV
    assert scn.frame_end >= 58 and scn.frame_current == 10
    assert A.STATE.takes == 1 and not A.STATE.recording

    assert A._do_rec_stop(scn) == (False, "Not recording")                 # idempotent
    first = act
    ok, _ = record_take(scn, cam)                                           # second take, maybe same second
    assert ok
    assert len([f for f in os.listdir(scn.arcam.output_dir) if f.endswith(".csv")]) == 2   # no overwrite
    assert cam.animation_data.action is not first                          # new action per take ...
    assert first.use_fake_user                                              # ... the first one is kept


def test_bake_subframe_mode():
    cam = FakeCam(base_looking_forward())
    scn = make_scene(cam, round_frames=False)
    fresh_state(scn)
    ok, _ = record_take(scn, cam, seconds=0.5)
    assert ok
    frames = [f for prop, f, _v, _a in cam.keys if prop == "location"]
    assert len(frames) == 31 and frames == sorted(frames) and frames[0] == 1.0
    assert any(abs(f - round(f)) > 1e-6 for f in frames)                   # true sub-frame keys


def test_bake_parented_camera():
    parent = NS(matrix_world=FM.Matrix.Translation((0, 0, 5)) @ FM.Matrix.Rotation(math.radians(90), 4, "Z"))
    cam = FakeCam(base_looking_forward(), parent=parent)
    cam.matrix_parent_inverse = FM.Matrix.Translation((1, 0, 0))            # some non-trivial inverse
    scn = make_scene(cam)
    fresh_state(scn)
    world = [(i / 24, (i / 24, 2.0, 1.0), (1.0, 0.0, 0.0, 0.0), 2) for i in range(25)]
    A._bake(scn, cam, world, 1, "parented", True)
    P = parent.matrix_world @ cam.matrix_parent_inverse
    locs = [(f, v) for prop, f, v, _a in cam.keys if prop == "location"]
    rots = {f: v for prop, f, v, _a in cam.keys if prop == "rotation_euler"}
    assert len(locs) == 25
    for f, v in locs:                                                       # parent @ local == world
        e = rots[f]
        local = FM.Matrix.Translation(v) @ (FM.Matrix.Rotation(e[2], 4, "Z") @ FM.Matrix.Rotation(e[1], 4, "Y")
                                             @ FM.Matrix.Rotation(e[0], 4, "X"))
        w = (P @ local).m
        t = (f - 1) / 24
        assert abs(w[0, 3] - t) < 1e-9 and abs(w[1, 3] - 2.0) < 1e-9 and abs(w[2, 3] - 1.0) < 1e-9, (f, w)
        assert abs(w[:3, :3] - FM.Matrix().m[:3, :3]).max() < 1e-9          # world rotation preserved


def test_detach_action():
    cam = FakeCam()
    assert A._detach_action(cam) is None                                    # no animation data
    cam.animation_data = NS(action=None)
    assert A._detach_action(cam) is None
    act = NS(name="MyMove", use_fake_user=False)
    cam.animation_data.action = act
    assert A._detach_action(cam) == "MyMove"
    assert cam.animation_data.action is None and act.use_fake_user          # kept, never lost


def test_record_guards():
    cam = FakeCam(base_looking_forward())
    scn = make_scene(cam)
    fresh_state(scn, running=False)
    assert A._do_rec_start(scn) == (False, "Start listening first")
    A.STATE.running = True
    assert A._do_rec_start(scn)[0]
    assert A._do_rec_start(scn) == (False, "Already recording")
    ok, msg = A._do_rec_stop(scn)                                           # no data arrived
    assert not ok and "Nothing recorded" in msg
    scn.arcam.camera = None
    assert A._do_rec_start(scn) == (False, "Pick a camera first")


def test_csv_roundtrip_and_import_operator():
    cam = FakeCam(base_looking_forward())
    scn = make_scene(cam)
    fresh_state(scn, running=False)
    take = [(i / 60, (i / 60, 0.0, 0.5), A._qnorm((1.0, 0.1 * i / 60, 0.0, 0.0)), 2) for i in range(61)]
    path = os.path.join(scn.arcam.output_dir, "t.csv")
    A._write_csv(path, take, 24.0, 1)
    back = A._read_csv_take(path)
    assert len(back) == 61
    assert max(abs(a[1][0] - b[1][0]) for a, b in zip(take, back)) < 1e-5
    assert max(abs(a[2][1] - b[2][1]) for a, b in zip(take, back)) < 1e-7

    reports = []
    op = A.ARCAM_OT_import_csv()
    op.report = lambda kind, msg: reports.append(msg)
    op.filepath = path
    scn.frame_current = 100
    assert op.execute(NS(scene=scn)) == {"FINISHED"}, reports
    frames = [f for prop, f, _v, _a in cam.keys if prop == "location"]
    assert frames[0] == 100.0 and frames[-1] == 124.0
    assert cam.animation_data.action.name == "import_t"

    A.STATE.running = True                                                  # refuses while live
    assert op.execute(NS(scene=scn)) == {"CANCELLED"}
    A.STATE.running = False
    op.filepath = os.path.join(scn.arcam.output_dir, "missing.csv")
    assert op.execute(NS(scene=scn)) == {"CANCELLED"} and "Bad CSV" in reports[-1]


# ------------------------------------------------------------------ operators / tick / handlers
def test_start_stop_operators():
    cam = FakeCam(base_looking_forward())
    cam.animation_data = NS(action=NS(name="OldTake", use_fake_user=False))
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("0.0.0.0", 0))
    free_port = probe.getsockname()[1]
    probe.close()
    scn = make_scene(cam, port=free_port)
    A.STATE.reset()
    A.STATE.running = False
    reports = []
    ctx = NS(scene=scn, area=None)
    start = A.ARCAM_OT_start()
    start.report = lambda kind, msg: reports.append(msg)
    assert start.execute(ctx) == {"FINISHED"}, reports
    try:
        assert A.STATE.running and A.STATE.thread.is_alive()
        assert bpy.app.timers.is_registered(A._tick)
        assert cam.animation_data.action is None                            # old take unassigned ...
        assert "OldTake" in A.STATE.last_msg                                # ... and the user is told
        assert scn.camera is cam

        busy = A.ARCAM_OT_start()                                           # same port, second listener
        busy.report = lambda kind, msg: reports.append(msg)
        A.STATE.running = False
        saved_sock, saved_thread = A.STATE.sock, A.STATE.thread
        assert busy.execute(ctx) == {"CANCELLED"} and "Cannot open UDP port" in reports[-1]
        A.STATE.sock, A.STATE.thread, A.STATE.running = saved_sock, saved_thread, True

        A.STATE.recording = True
        stop = A.ARCAM_OT_stop()
        stop.report = lambda kind, msg: reports.append(msg)
        assert stop.execute(ctx) == {"CANCELLED"}                           # refuses while recording
        A.STATE.recording = False
        assert stop.execute(ctx) == {"FINISHED"}
        assert not A.STATE.running and A.STATE.sock is None and A.STATE.thread is None
        assert A._tick() is None                                            # timer ends itself
    finally:
        A._stop_listener()
        _TIMERS.clear()


def test_tick_commands_and_status():
    cam = FakeCam(base_looking_forward())
    scn = make_scene(cam)
    fresh_state(scn)
    st = A.STATE
    sent = []
    st.sock = NS(sendto=lambda data, addr: sent.append((data, addr)))
    st.peer = ("10.0.0.5", 51234)
    st.cmds.extend(["rec_start"])
    st.queue.extend([pkt(1.0), pkt(1.02, pos=(0, 0, -1))])
    assert A._tick() == 1.0 / 60.0
    assert st.recording and len(st.samples) == 2
    assert sent and sent[0][1] == ("10.0.0.5", 51234) and sent[0][0].startswith(b"ST,1,2,")
    st.cmds.extend(["rec_stop"])
    st.last_status = 0
    A._tick()
    assert not st.recording and st.takes == 1
    st.last_status = 0
    A._tick()
    assert sent[-1][0].startswith(b"ST,0,0,0.0,1"), sent[-1]
    st.cmds.extend(["recenter"])
    A._tick()
    assert st.last_msg == "Recentered from phone"
    st.sock = None
    st.running = False
    assert A._tick() is None and st.timer_on is False


def test_tick_follows_renamed_scene():
    cam = FakeCam(base_looking_forward())
    scn = make_scene(cam)
    fresh_state(scn)
    A.STATE.scene_name = "OldName"                                          # scene was renamed
    bpy.context.scene = scn
    A.STATE.queue.extend([pkt(1.0), pkt(1.02, pos=(0, 0, -1))])
    A._tick()
    assert A.STATE.scene_name == "Scene" and cam_pos(cam) == [0.0, 1.0, 0.0]
    bpy.context.scene = None


def test_tick_survives_errors():
    cam = FakeCam(base_looking_forward())
    scn = make_scene(cam)
    fresh_state(scn)
    A.STATE.queue.append(("bad",))                                          # malformed entry
    with contextlib.redirect_stderr(io.StringIO()):
        assert A._tick() == 1.0 / 60.0                                      # timer must not die
    assert A.STATE.error


def test_file_load_saves_take_and_stops():
    cam = FakeCam(base_looking_forward())
    scn = make_scene(cam)
    fresh_state(scn)
    s, _c, _dest = udp_pair()
    A.STATE.sock = s
    ok, _ = A._do_rec_start(scn)
    assert ok
    for i in range(30):
        A._process(A.STATE, scn.arcam, cam, [pkt(2.0 + i / 60, pos=(0, 0, -i / 60))])
    with contextlib.redirect_stdout(io.StringIO()):
        A._on_load_pre("other.blend")
    assert not A.STATE.running and not A.STATE.recording and A.STATE.sock is None
    folder = os.path.join(FAKE_HOME, "ARCam_takes")
    saved = [f for f in os.listdir(folder) if f.endswith("_unsaved.csv")]
    assert len(saved) == 1
    assert len(open(os.path.join(folder, saved[0])).read().splitlines()) == 31
    assert A._emergency_save() is None                                      # nothing left to save


def test_register_unregister_handlers():
    reg = []
    bpy.utils = NS(register_class=lambda c: reg.append(c), unregister_class=lambda c: reg.remove(c))
    bpy.types.Scene = type("Scene", (), {})
    bpy.app.handlers.exit_pre = []                                          # Blender 5.1+ has it
    A.register()
    assert A._on_load_pre in bpy.app.handlers.load_pre and A._on_load_pre in bpy.app.handlers.exit_pre
    assert len(reg) == len(A._classes)
    A.unregister()
    assert A._on_load_pre not in bpy.app.handlers.load_pre and A._on_load_pre not in bpy.app.handlers.exit_pre
    assert reg == [] and not hasattr(bpy.types.Scene, "arcam")
    del bpy.app.handlers.exit_pre                                           # older Blender: no exit_pre
    A.register()
    A.unregister()


def test_icon_helper_without_blender():
    assert A._icon("REC") == "REC"                                          # no icon list available: pass-through


def test_end_to_end_phone_record_loop():
    """fake_phone -> real UDP -> listener thread -> timer -> camera/recording -> status back to the phone."""
    cam = FakeCam(base_looking_forward())
    scn = make_scene(cam)
    fresh_state(scn)
    st = A.STATE
    s, _c, dest = udp_pair()
    st.sock = s
    ev, th = run_listener(s)
    proc = subprocess.Popen([sys.executable, os.path.join(ROOT, "tools", "fake_phone.py"), "127.0.0.1",
                             str(dest[1]), "--record", "1.5", "--noise", "2"],
                            stdout=subprocess.PIPE, text=True)
    end = time.time() + 8.0
    while time.time() < end and proc.poll() is None:
        A._tick()
        time.sleep(1 / 60)
    out = proc.communicate(timeout=5)[0]
    ev.set(); th.join(); s.close()
    st.sock = None
    print("   phone saw:", " | ".join(line for line in out.splitlines() if "ST," in line)[:120])
    assert "-> CMD,rec_start" in out and "-> CMD,rec_stop" in out
    assert "<- ST,1," in out, out                                          # phone was told "recording"
    assert "<- ST,0,0,0.0,1" in out, out                                   # ... and "saved, 1 take"
    csvs = [f for f in os.listdir(scn.arcam.output_dir) if f.endswith(".csv")]
    assert len(csvs) == 1
    n = len(open(os.path.join(scn.arcam.output_dir, csvs[0])).read().splitlines()) - 1
    assert 60 < n < 130, n                                                  # ~1.5 s at 60 Hz


if __name__ == "__main__":
    failed = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("ok  ", name)
            except Exception as e:  # report all failures, not just the first
                failed += 1
                import traceback
                traceback.print_exc()
                print("FAIL", name, repr(e))
    print("\nALL PASSED" if not failed else f"\n{failed} FAILED")
    sys.exit(1 if failed else 0)
