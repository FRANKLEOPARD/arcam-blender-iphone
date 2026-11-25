bl_info = {
    "name": "ARCam - iPhone Virtual Camera (ARKit over UDP)",
    "author": "ARCam contributors",
    "version": (2, 0, 0),
    "blender": (3, 0, 0),
    "location": "3D Viewport > Sidebar (N) > ARCam",
    "description": "Drive a Blender camera live from an iPhone (ARKit pose over UDP) and record takes",
    "category": "Animation",
}

import bpy
import collections
import csv
import math
import os
import socket
import textwrap
import threading
import time
import traceback
from datetime import datetime

from bpy.props import BoolProperty, FloatProperty, IntProperty, PointerProperty, StringProperty
from mathutils import Matrix, Quaternion, Vector

# ARKit world (X right, Y up, Z toward viewer) -> Blender world (X right, Y forward, Z up)
# = rotation of +90 degrees about X. Camera local axes are identical in both
# (looks down -Z, up is +Y), so only the world needs converting.
_C = Matrix.Rotation(math.radians(90.0), 4, "X")


# --------------------------------------------------------------------------
# Pure-Python helpers (no bpy / mathutils, so they are unit-testable)
# --------------------------------------------------------------------------
def _qdot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]


def _qmul(a, b):
    return (a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0])


def _qnorm(q):
    n = math.sqrt(_qdot(q, q))
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def _qslerp(a, b, t):
    """Shortest-path spherical interpolation of unit quaternions (w, x, y, z)."""
    d = _qdot(a, b)
    if d < 0.0:
        b = (-b[0], -b[1], -b[2], -b[3])
        d = -d
    if d > 0.9995:
        return _qnorm(tuple(a[i] + t * (b[i] - a[i]) for i in range(4)))
    th0 = math.acos(min(1.0, d))
    s0 = math.sin((1.0 - t) * th0) / math.sin(th0)
    s1 = math.sin(t * th0) / math.sin(th0)
    return _qnorm(tuple(s0 * a[i] + s1 * b[i] for i in range(4)))


class OneEuroPose:
    """Speed-adaptive low-pass filter (One Euro filter) for a position + quaternion.

    Slow movement -> strong smoothing (removes jitter); fast movement -> the cut-off rises,
    so there is little lag. `s` is the user's 0..1 smoothing amount.
    """
    BETA_POS = 4.0      # Hz of extra cut-off per m/s
    BETA_ROT = 2.0      # Hz of extra cut-off per rad/s
    D_CUTOFF = 1.0      # Hz, smoothing of the speed estimate

    def __init__(self):
        self.reset()

    def reset(self):
        self.t = None
        self.raw_loc = None
        self.raw_q = None
        self.loc = None
        self.q = None
        self.dv = [0.0, 0.0, 0.0]   # low-passed SIGNED velocity (noise cancels out, unlike a magnitude)
        self.dw = [0.0, 0.0, 0.0]   # low-passed angular velocity vector (rad/s)

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def _snap(self, t, loc, q):
        self.t = t
        self.raw_loc, self.raw_q = loc, q
        self.loc, self.q = loc, q
        self.dv = [0.0, 0.0, 0.0]
        self.dw = [0.0, 0.0, 0.0]
        return loc, q

    def update(self, t, loc, q, s, jump=1.5):
        s = min(max(s, 0.0), 1.0)
        # first sample, long gap, clock going backwards: start over
        if self.t is None or t <= self.t or t - self.t > 0.5:
            return self._snap(t, loc, q)
        dt = min(max(t - self.t, 1.0 / 240.0), 0.25)
        dist = math.sqrt(sum((loc[i] - self.raw_loc[i]) ** 2 for i in range(3)))
        if dist > jump:   # ARKit relocalization / tracking jump: snap instead of smearing
            return self._snap(t, loc, q)

        a_d = self._alpha(self.D_CUTOFF, dt)
        pos_min = 8.0 * (0.5 / 8.0) ** s
        rot_min = 10.0 * (0.4 / 10.0) ** s

        for i in range(3):
            self.dv[i] += a_d * ((loc[i] - self.raw_loc[i]) / dt - self.dv[i])
        speed = math.sqrt(sum(v * v for v in self.dv))
        a = self._alpha(pos_min + self.BETA_POS * speed, dt)
        new_loc = tuple(self.loc[i] + a * (loc[i] - self.loc[i]) for i in range(3))

        dq = _qmul(q, (self.raw_q[0], -self.raw_q[1], -self.raw_q[2], -self.raw_q[3]))
        if dq[0] < 0.0:
            dq = (-dq[0], -dq[1], -dq[2], -dq[3])
        for i in range(3):
            self.dw[i] += a_d * (2.0 * dq[i + 1] / dt - self.dw[i])
        ang_speed = math.sqrt(sum(v * v for v in self.dw))
        ar = self._alpha(rot_min + self.BETA_ROT * ang_speed, dt)
        new_q = _qslerp(self.q, q, ar)

        self.t = t
        self.raw_loc, self.raw_q = loc, q
        self.loc, self.q = new_loc, new_q
        return new_loc, new_q


def _smooth_take(samples, sigma):
    """Zero-lag Gaussian smoothing of a recorded take (offline, so no delay).

    samples: list of (time_s, (x, y, z), (w, x, y, z), tracking). sigma in seconds.
    """
    n = len(samples)
    if sigma <= 1e-6 or n < 3:
        return samples
    samples = sorted(samples, key=lambda smp: smp[0])
    t = [smp[0] for smp in samples]
    reach = 3.0 * sigma + 1e-9   # tiny tolerance so both window edges treat rounding the same way
    out = []
    lo = hi = 0
    for i in range(n):
        ti = t[i]
        while t[lo] < ti - reach:
            lo += 1
        if hi < i:
            hi = i
        while hi + 1 < n and t[hi + 1] <= ti + reach:
            hi += 1
        qref = samples[i][2]
        wsum = 0.0
        loc = [0.0, 0.0, 0.0]
        q = [0.0, 0.0, 0.0, 0.0]
        for j in range(lo, hi + 1):
            d = (t[j] - ti) / sigma
            w = math.exp(-0.5 * d * d)
            qj = samples[j][2]
            sign = 1.0 if _qdot(qj, qref) >= 0.0 else -1.0
            wsum += w
            for k in range(3):
                loc[k] += w * samples[j][1][k]
            for k in range(4):
                q[k] += w * sign * qj[k]
        out.append((ti, (loc[0] / wsum, loc[1] / wsum, loc[2] / wsum),
                    _qnorm(tuple(q)), samples[i][3]))
    return out


def _resample(samples, fps, start_frame):
    """Interpolate a take at exact whole scene frames (no timing bias).

    samples: sorted list of (time_s, loc, quat_wxyz, tracking), time_s >= 0.
    Returns a list of (frame, loc, quat_wxyz). Gaps (e.g. tracking lost) are bridged linearly.
    """
    n = len(samples)
    if n == 0:
        return []
    out = []
    f = math.ceil(start_frame + samples[0][0] * fps - 1e-6)
    f_end = start_frame + samples[-1][0] * fps + 1e-6
    j = 0
    while f <= f_end:
        t = (f - start_frame) / fps
        while j + 1 < n and samples[j + 1][0] <= t:
            j += 1
        a = samples[j]
        b = samples[min(j + 1, n - 1)]
        span = b[0] - a[0]
        u = 0.0 if span <= 1e-12 else min(1.0, max(0.0, (t - a[0]) / span))
        loc = tuple(a[1][k] + u * (b[1][k] - a[1][k]) for k in range(3))
        out.append((float(f), loc, _qslerp(a[2], b[2], u)))
        f += 1
    return out


# --------------------------------------------------------------------------
# Shared state (the network thread only touches `queue`, `cmds`, `peer`, etc.
# under the lock. It NEVER touches bpy.)
# --------------------------------------------------------------------------
class _State:
    def __init__(self):
        self.lock = threading.Lock()
        self.thread = None
        self.stop_evt = threading.Event()
        self.sock = None
        self.running = False
        self.timer_on = False
        self.reset()
        self.ips = []

    def reset(self):
        self.latest = None      # (ts, (px,py,pz), (qw,qx,qy,qz), tracking)
        self.counter = 0
        self.queue = collections.deque(maxlen=1200)   # every valid packet, drained by the timer
        self.cmds = collections.deque(maxlen=32)      # commands sent by the phone
        self.peer = None        # (ip, port) of the phone, to send status back
        self.last_rx = 0.0
        self.last_ts = None
        self.trk = 0
        self.ref = None
        self.base = Matrix.Identity(4)
        self.recenter = False
        self.filt = OneEuroPose()
        self.recording = False
        self.samples = []
        self.rec_t0 = None
        self.rec_wall0 = 0.0
        self.rec_start_frame = 1
        self.rec_fps = 24.0
        self.takes = 0
        self.scene_name = ""
        self.last_redraw = 0.0
        self.last_status = 0.0
        self.last_msg = ""
        self.error = ""


STATE = _State()


# --------------------------------------------------------------------------
# Networking
# --------------------------------------------------------------------------
def _parse(data):
    """'AR1,seq,ts,px,py,pz,qx,qy,qz,qw,tracking' -> tuple or None."""
    try:
        parts = data.decode("ascii", "ignore").strip().split(",")
        if len(parts) < 11 or parts[0] != "AR1":
            return None
        seq = int(parts[1])
        ts = float(parts[2])
        px, py, pz, qx, qy, qz, qw = [float(x) for x in parts[3:10]]
        trk = int(parts[10])
    except (ValueError, UnicodeError):
        return None
    if trk not in (0, 1, 2):
        return None
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-6 or not all(math.isfinite(v) for v in (px, py, pz, n, ts)):
        return None
    return seq, ts, (px, py, pz), (qw / n, qx / n, qy / n, qz / n), trk


_COMMANDS = ("recenter", "rec_start", "rec_stop")


def _parse_cmd(data):
    """'CMD,recenter' -> 'recenter' (only known commands) or None."""
    try:
        parts = data.decode("ascii", "ignore").strip().split(",")
    except UnicodeError:
        return None
    if len(parts) == 2 and parts[0] == "CMD" and parts[1] in _COMMANDS:
        return parts[1]
    return None


def _listen(sock, stop_evt):
    st = STATE
    while not stop_evt.is_set():
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            continue
        except ConnectionResetError:
            continue  # Windows can raise WSAECONNRESET (10054) on UDP sockets: not fatal
        except OSError:
            break     # socket closed by _stop_listener
        pkt = _parse(data)
        if pkt is None:
            cmd = _parse_cmd(data)
            if cmd is not None:
                with st.lock:
                    st.cmds.append(cmd)
                    st.peer = addr
            continue
        _seq, ts, pos, quat, trk = pkt
        with st.lock:
            # drop old/out-of-order UDP packets (accept a big backwards jump = phone restarted)
            if st.last_ts is not None and ts <= st.last_ts and ts > st.last_ts - 10.0:
                continue
            st.last_ts = ts
            st.latest = (ts, pos, quat, trk)
            st.queue.append((ts, pos, quat, trk))
            st.peer = addr
            st.trk = trk
            st.counter += 1
            st.last_rx = time.time()


def _local_ips(timeout=0.3):
    """IPv4 addresses of this computer, the most likely one first. Never blocks for long
    (socket.gethostbyname_ex can hang for seconds on some Macs)."""
    primary = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("10.255.255.255", 1))  # no packet is actually sent
            primary = s.getsockname()[0]
        finally:
            s.close()
    except OSError:
        pass

    found = []

    def _lookup():
        try:
            found.extend(socket.gethostbyname_ex(socket.gethostname())[2])
        except (OSError, UnicodeError):
            pass

    th = threading.Thread(target=_lookup, daemon=True)
    th.start()
    th.join(timeout)
    others = sorted({ip for ip in list(found)
                     if not ip.startswith(("127.", "169.254.", "0.")) and ip != primary})
    ips = ([primary] if primary and not primary.startswith(("127.", "0.")) else []) + others
    return ips


def _stop_listener():
    st = STATE
    st.running = False
    st.stop_evt.set()
    if st.thread is not None:
        st.thread.join(timeout=1.5)
        st.thread = None
    if st.sock is not None:
        try:
            st.sock.close()
        except OSError:
            pass
        st.sock = None


# --------------------------------------------------------------------------
# Math / camera driving
# --------------------------------------------------------------------------
def _arkit_to_blender(pos, quat_wxyz, scale, lock_pos, roll):
    m = Quaternion(quat_wxyz).to_matrix().to_4x4()
    if lock_pos:
        m.translation = Vector((0.0, 0.0, 0.0))
    else:
        m.translation = Vector(pos) * scale
    if roll:
        m = m @ Matrix.Rotation(roll, 4, "Z")
    return _C @ m


def _cam_base(cam):
    loc, rot, _s = cam.matrix_world.decompose()
    return Matrix.Translation(loc) @ rot.to_matrix().to_4x4()


def _process(st, p, cam, pkts):
    """Handle every packet received since the last tick (record all, filter all, move camera once)."""
    last = None
    for ts, pos, quat, trk in pkts:
        if trk == 0:  # ARKit says tracking not available: ignore junk pose
            continue
        M = _arkit_to_blender(pos, quat, p.pos_scale, p.lock_pos, p.roll_offset)
        if st.ref is None or st.recenter:
            st.ref = st.base @ M.inverted()
            st.recenter = False
            st.filt.reset()
        loc, rot, _s = (st.ref @ M).decompose()
        loc_t = (loc.x, loc.y, loc.z)
        q_t = (rot.w, rot.x, rot.y, rot.z)

        if st.recording:  # always raw
            if st.rec_t0 is None:
                st.rec_t0 = ts
            st.samples.append((ts - st.rec_t0, loc_t, q_t, trk))

        if p.smoothing > 0.001:
            last = st.filt.update(ts, loc_t, q_t, p.smoothing, jump=1.5 * max(1.0, p.pos_scale))
        else:
            last = (loc_t, q_t)
    if last is not None:
        loc_f, q_f = last
        cam.matrix_world = Matrix.Translation(Vector(loc_f)) @ Quaternion(q_f).to_matrix().to_4x4()
        return True
    return False


def _send_status(st):
    """Tell the phone what Blender is doing: ST,recording,samples,elapsed_s,takes"""
    if st.sock is None or st.peer is None:
        return
    n = len(st.samples) if st.recording else 0
    el = (time.time() - st.rec_wall0) if st.recording else 0.0
    msg = f"ST,{1 if st.recording else 0},{n},{el:.1f},{st.takes}"
    try:
        st.sock.sendto(msg.encode("ascii"), st.peer)
    except OSError:
        pass


def _run_command(st, scn, cmd):
    if cmd == "recenter":
        st.recenter = True
        st.last_msg = "Recentered from phone"
    elif cmd == "rec_start":
        st.last_msg = _do_rec_start(scn)[1]
    elif cmd == "rec_stop":
        st.last_msg = _do_rec_stop(scn)[1]


def _redraw():
    try:
        for w in bpy.context.window_manager.windows:
            for a in w.screen.areas:
                if a.type == "VIEW_3D":
                    a.tag_redraw()
    except Exception:
        pass


def _find_scene(st):
    scn = bpy.data.scenes.get(st.scene_name)
    if scn is None:  # scene renamed while listening: follow the active scene
        try:
            scn = bpy.context.scene
            if scn is not None:
                st.scene_name = scn.name
        except Exception:
            scn = None
    return scn


def _tick():
    st = STATE
    if not st.running:
        st.timer_on = False
        return None
    try:
        scn = _find_scene(st)
        if scn is not None:
            with st.lock:
                pkts = list(st.queue)
                st.queue.clear()
                cmds = list(st.cmds)
                st.cmds.clear()
            for c in cmds:
                _run_command(st, scn, c)
            p = scn.arcam
            moved = bool(pkts) and p.camera is not None and _process(st, p, p.camera, pkts)
            now = time.time()
            if now - st.last_status > 0.25:
                st.last_status = now
                _send_status(st)
            if moved or now - st.last_redraw > 0.25:
                st.last_redraw = now
                _redraw()
    except Exception as e:  # never let the timer die silently
        st.error = str(e)
        traceback.print_exc()
    return 1.0 / 60.0


# --------------------------------------------------------------------------
# Recording helpers
# --------------------------------------------------------------------------
def _out_dir(p):
    d = (p.output_dir if p is not None else "") or ""
    if d.startswith("//") and not bpy.data.filepath:
        d = ""
    d = bpy.path.abspath(d) if d else ""
    if not d or not os.path.isabs(d):
        d = os.path.join(os.path.expanduser("~"), "ARCam_takes")
    os.makedirs(d, exist_ok=True)
    return d


def _unique_csv(folder, name):
    path = os.path.join(folder, name + ".csv")
    k = 2
    while os.path.exists(path):  # two takes in the same second must not overwrite each other
        path = os.path.join(folder, f"{name}_{k}.csv")
        k += 1
    return path


def _write_csv(path, samples, fps, start_frame):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "frame", "tracking", "px", "py", "pz",
                    "qw", "qx", "qy", "qz", "rx_deg", "ry_deg", "rz_deg"])
        prev = None
        for dt, loc, q, trk in samples:
            quat = Quaternion(q)
            eul = quat.to_euler("XYZ", prev) if prev is not None else quat.to_euler("XYZ")
            prev = eul
            w.writerow([f"{dt:.5f}", f"{start_frame + dt * fps:.3f}", trk]
                       + [f"{v:.6f}" for v in loc]
                       + [f"{v:.8f}" for v in q]
                       + [f"{math.degrees(a):.5f}" for a in (eul.x, eul.y, eul.z)])


def _detach_action(cam):
    """Unassign the camera's current action (kept with a fake user so it is never lost).
    Without this, the old take snaps the camera back whenever the frame changes."""
    ad = getattr(cam, "animation_data", None)
    act = getattr(ad, "action", None) if ad is not None else None
    if act is None:
        return None
    act.use_fake_user = True
    ad.action = None
    return act.name


def _bake(scn, cam, samples, start_frame, name, round_frames):
    """Keyframe the samples on `cam` in a NEW action (previous takes are kept).

    samples are in world space; if the camera has a parent they are converted to its local space.
    keyframe_insert() creates the action itself, which works with every Blender version
    including the slotted Actions introduced in 4.4.
    """
    fps = scn.render.fps / scn.render.fps_base
    samples = sorted(samples, key=lambda smp: smp[0])
    if round_frames:
        keys = _resample(samples, fps, start_frame)
    else:
        keys = [(start_frame + dt * fps, loc, q) for dt, loc, q, _trk in samples]
    if not keys:
        return 0

    pinv = None
    if getattr(cam, "parent", None) is not None:
        pinv = (cam.parent.matrix_world @ cam.matrix_parent_inverse).inverted()

    ad = cam.animation_data or cam.animation_data_create()
    _detach_action(cam)
    cam.rotation_mode = "XYZ"

    prev = None
    count = 0
    last_f = None
    for f, loc, q in keys:
        if last_f is not None and f <= last_f:   # duplicate / unsorted frame: skip
            continue
        last_f = f
        if pinv is not None:
            local = pinv @ (Matrix.Translation(Vector(loc)) @ Quaternion(q).to_matrix().to_4x4())
            lv, lq, _s = local.decompose()
            loc = (lv.x, lv.y, lv.z)
            q = (lq.w, lq.x, lq.y, lq.z)
        quat = Quaternion(q)
        eul = quat.to_euler("XYZ", prev) if prev is not None else quat.to_euler("XYZ")
        prev = eul
        cam.location = loc
        cam.rotation_euler = eul
        cam.keyframe_insert("location", frame=f)
        cam.keyframe_insert("rotation_euler", frame=f)
        count += 1

    act = getattr(cam.animation_data or ad, "action", None)
    if act is not None:
        act.name = name
        act.use_fake_user = True
    scn.frame_end = max(scn.frame_end, int(math.ceil(last_f)))
    scn.frame_set(int(start_frame))
    return count


def _emergency_save():
    """Called before a new file is loaded or the add-on is disabled: never lose a take."""
    st = STATE
    if not st.recording:
        return None
    st.recording = False
    samples, st.samples = st.samples, []
    if len(samples) < 2:
        return None
    try:
        folder = os.path.join(os.path.expanduser("~"), "ARCam_takes")
        os.makedirs(folder, exist_ok=True)
        path = _unique_csv(folder, datetime.now().strftime("take_%Y%m%d_%H%M%S_unsaved"))
        _write_csv(path, samples, st.rec_fps, st.rec_start_frame)
        print("ARCam: recording interrupted, take saved to", path)
        return path
    except OSError as e:
        print("ARCam: could not save the interrupted take:", e)
        return None


def _do_rec_start(scn):
    st = STATE
    p = scn.arcam
    if not st.running:
        return False, "Start listening first"
    if st.recording:
        return False, "Already recording"
    if p.camera is None:
        return False, "Pick a camera first"
    st.samples = []
    st.rec_t0 = None
    st.rec_wall0 = time.time()
    st.rec_start_frame = scn.frame_current
    st.rec_fps = scn.render.fps / scn.render.fps_base
    st.recording = True
    return True, "Recording"


def _do_rec_stop(scn):
    st = STATE
    p = scn.arcam
    if not st.recording:
        return False, "Not recording"
    st.recording = False
    samples, st.samples = st.samples, []
    if len(samples) < 2 or p.camera is None:
        return False, "Nothing recorded (no data received?)"
    fps = scn.render.fps / scn.render.fps_base
    name = datetime.now().strftime("take_%Y%m%d_%H%M%S")
    note = ""
    try:
        path = _unique_csv(_out_dir(p), name)
        name = os.path.splitext(os.path.basename(path))[0]   # action name = CSV name
        _write_csv(path, samples, fps, st.rec_start_frame)   # raw data is always saved first
        note = f" CSV: {path}"
    except OSError as e:
        note = f" (CSV not saved: {e})"
    n = _bake(scn, p.camera, _smooth_take(samples, p.take_smooth), st.rec_start_frame, name, p.round_frames)
    st.takes += 1
    return True, f"{name}: {len(samples)} samples -> {n} keys.{note}"


# --------------------------------------------------------------------------
# Properties
# --------------------------------------------------------------------------
def _is_camera(self, obj):
    return obj.type == "CAMERA"


def _on_anchor_change(self, context):
    """Move scale / rotation-only / roll changed while live: re-anchor so the camera does not jump."""
    st = STATE
    if st.running and self.camera is not None:
        st.base = _cam_base(self.camera)
        st.recenter = True


class ARCamProps(bpy.types.PropertyGroup):
    camera: PointerProperty(name="Camera", type=bpy.types.Object, poll=_is_camera)
    port: IntProperty(name="UDP port", default=9000, min=1024, max=65535)
    pos_scale: FloatProperty(
        name="Move scale", default=1.0, min=0.001, soft_max=100.0, update=_on_anchor_change,
        description="1 = real metres. >1 makes a small walk cover a bigger set")
    lock_pos: BoolProperty(name="Rotation only", default=False, update=_on_anchor_change,
                           description="Ignore position, use only phone rotation (tripod/pan-tilt mode)")
    smoothing: FloatProperty(name="Live smoothing", default=0.5, min=0.0, max=1.0,
                             description="Adaptive smoothing of the viewport camera: strong when the phone is "
                                         "nearly still, almost none when it moves fast. 0 = off. Recording stays raw")
    take_smooth: FloatProperty(name="Take smoothing (s)", default=0.03, min=0.0, max=0.5,
                               description="Zero-lag smoothing applied to the keyframes when a take is baked, "
                                           "in seconds (0 = off). The CSV always keeps the raw data")
    roll_offset: FloatProperty(name="Roll offset", default=0.0, subtype="ANGLE", update=_on_anchor_change,
                               min=-math.pi, max=math.pi,
                               description="Fine-tune if the view is rolled (try +/-90)")
    output_dir: StringProperty(name="Take folder", subtype="DIR_PATH", default="//ARCam_takes")
    round_frames: BoolProperty(name="Key whole frames", default=True,
                               description="One key per scene frame, interpolated at the exact frame time. "
                                           "Off = one key per phone sample (sub-frame keys)")


# --------------------------------------------------------------------------
# Operators
# --------------------------------------------------------------------------
class ARCAM_OT_start(bpy.types.Operator):
    bl_idname = "arcam.start"
    bl_label = "Start listening"
    bl_description = "Open the UDP port and let the phone drive the camera"

    def execute(self, context):
        st = STATE
        p = context.scene.arcam
        if p.camera is None:
            self.report({"ERROR"}, "Pick a camera first")
            return {"CANCELLED"}
        if st.running:
            return {"FINISHED"}
        s = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind(("0.0.0.0", p.port))
        except OSError as e:
            if s is not None:
                try:
                    s.close()
                except OSError:
                    pass
            hint = ""
            if getattr(e, "winerror", None) == 10013 or getattr(e, "errno", None) == 13:
                hint = " (port reserved or blocked: try another port, e.g. 9500)"
            elif getattr(e, "winerror", None) == 10048 or getattr(e, "errno", None) in (48, 98):
                hint = " (already used by another program or Blender window)"
            self.report({"ERROR"}, f"Cannot open UDP port {p.port}: {e}{hint}")
            return {"CANCELLED"}
        s.settimeout(0.25)
        st.reset()
        st.sock = s
        st.scene_name = context.scene.name
        st.base = _cam_base(p.camera)          # pose BEFORE detaching = what you see now
        detached = _detach_action(p.camera)
        if detached:
            st.last_msg = f"Previous animation '{detached}' unassigned (kept) so the camera follows the phone"
        st.ips = _local_ips()
        st.stop_evt = threading.Event()
        st.thread = threading.Thread(target=_listen, args=(s, st.stop_evt), daemon=True)
        st.thread.start()
        st.running = True
        if not bpy.app.timers.is_registered(_tick):
            bpy.app.timers.register(_tick, first_interval=0.0)
        st.timer_on = True
        context.scene.camera = p.camera
        try:
            context.area.spaces.active.region_3d.view_perspective = "CAMERA"
        except Exception:
            pass
        return {"FINISHED"}


class ARCAM_OT_stop(bpy.types.Operator):
    bl_idname = "arcam.stop"
    bl_label = "Stop"
    bl_description = "Close the UDP port (stop the recording first)"

    def execute(self, context):
        if STATE.recording:
            self.report({"WARNING"}, "Stop recording first (so the take is saved)")
            return {"CANCELLED"}
        _stop_listener()
        return {"FINISHED"}


class ARCAM_OT_recenter(bpy.types.Operator):
    bl_idname = "arcam.recenter"
    bl_label = "Recenter to start"
    bl_description = "Snap the camera back to its start pose using the phone's current pose as the reference"

    def execute(self, context):
        if not STATE.running:
            self.report({"INFO"}, "Not listening")
            return {"CANCELLED"}
        STATE.recenter = True
        return {"FINISHED"}


class ARCAM_OT_set_start(bpy.types.Operator):
    bl_idname = "arcam.set_start"
    bl_label = "Set start = camera now"
    bl_description = "Use the camera's current pose as the new start point (no jump)"

    def execute(self, context):
        p = context.scene.arcam
        if not STATE.running or p.camera is None:
            self.report({"INFO"}, "Start listening first")
            return {"CANCELLED"}
        STATE.base = _cam_base(p.camera)
        STATE.recenter = True
        return {"FINISHED"}


class ARCAM_OT_refresh_ip(bpy.types.Operator):
    bl_idname = "arcam.refresh_ip"
    bl_label = "Refresh IP"
    bl_description = "Look up this computer's IP addresses again"

    def execute(self, context):
        STATE.ips = _local_ips()
        return {"FINISHED"}


class ARCAM_OT_rec_start(bpy.types.Operator):
    bl_idname = "arcam.rec_start"
    bl_label = "Record take"
    bl_description = "Start recording the phone's movement"

    def execute(self, context):
        ok, msg = _do_rec_start(context.scene)
        STATE.last_msg = msg
        self.report({"INFO"} if ok else {"ERROR"}, msg)
        return {"FINISHED"} if ok else {"CANCELLED"}


class ARCAM_OT_rec_stop(bpy.types.Operator):
    bl_idname = "arcam.rec_stop"
    bl_label = "Stop & save take"
    bl_description = "Stop recording, write the CSV and bake keyframes in a new Action"

    def execute(self, context):
        ok, msg = _do_rec_stop(context.scene)
        STATE.last_msg = msg
        self.report({"INFO"} if ok else {"WARNING"}, msg)
        return {"FINISHED"} if ok else {"CANCELLED"}


def _read_csv_take(path):
    samples = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            samples.append((float(r["time_s"]),
                            (float(r["px"]), float(r["py"]), float(r["pz"])),
                            _qnorm((float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"]))),
                            int(r["tracking"])))
    samples.sort(key=lambda smp: smp[0])
    return samples


class ARCAM_OT_import_csv(bpy.types.Operator):
    bl_idname = "arcam.import_csv"
    bl_label = "Import take (CSV)"
    bl_description = "Re-bake a recorded CSV take onto the camera, starting at the current frame"

    filepath: StringProperty(subtype="FILE_PATH")
    filter_glob: StringProperty(default="*.csv", options={"HIDDEN"})

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {"RUNNING_MODAL"}

    def execute(self, context):
        scn = context.scene
        p = scn.arcam
        if p.camera is None:
            self.report({"ERROR"}, "Pick a camera first")
            return {"CANCELLED"}
        if STATE.running:
            self.report({"WARNING"}, "Press Stop (listening) first, otherwise the live phone overrides the take")
            return {"CANCELLED"}
        try:
            samples = _read_csv_take(self.filepath)
        except (OSError, KeyError, ValueError, TypeError) as e:
            self.report({"ERROR"}, f"Bad CSV: {e}")
            return {"CANCELLED"}
        if len(samples) < 2:
            self.report({"ERROR"}, "CSV has no samples")
            return {"CANCELLED"}
        name = "import_" + os.path.splitext(os.path.basename(self.filepath))[0]
        n = _bake(scn, p.camera, _smooth_take(samples, p.take_smooth), scn.frame_current, name, p.round_frames)
        self.report({"INFO"}, f"Imported {len(samples)} samples -> {n} keys")
        return {"FINISHED"}


# --------------------------------------------------------------------------
# UI
# --------------------------------------------------------------------------
_ICONS = None


def _icon(name):
    """Return `name` if this Blender version has that icon, else 'NONE' (an unknown icon breaks the panel)."""
    global _ICONS
    if _ICONS is None:
        try:
            _ICONS = set(bpy.types.UILayout.bl_rna.functions["prop"].parameters["icon"].enum_items.keys())
        except Exception:
            _ICONS = set()
    if not _ICONS:
        return name
    return name if name in _ICONS else "NONE"


class ARCAM_PT_main(bpy.types.Panel):
    bl_label = "ARCam Virtual Camera"
    bl_idname = "ARCAM_PT_main"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "ARCam"

    def draw(self, context):
        L = self.layout
        p = context.scene.arcam
        st = STATE

        box = L.box()
        box.prop(p, "camera")
        row = box.row()
        row.enabled = not st.running
        row.prop(p, "port")
        if st.running:
            box.operator("arcam.stop", icon=_icon("PAUSE"))
        else:
            box.operator("arcam.start", icon=_icon("PLAY"))

        if st.running:
            fresh = st.last_rx and (time.time() - st.last_rx) < 1.0
            if fresh:
                names = {0: "not available", 1: "limited", 2: "normal"}
                L.label(text="Receiving - tracking " + names.get(st.trk, "?"),
                        icon=_icon("CHECKMARK") if st.trk == 2 else _icon("ERROR"))
            else:
                L.label(text="Listening... no data", icon=_icon("TIME"))
        row = L.row()
        row.label(text="PC IP: " + (", ".join(st.ips) if st.ips else "press Start"))
        row.operator("arcam.refresh_ip", text="", icon=_icon("FILE_REFRESH"))

        box = L.box()
        box.label(text="Camera feel")
        col = box.column(align=True)
        col.prop(p, "smoothing", slider=True)
        col.prop(p, "pos_scale")
        col.prop(p, "roll_offset")
        col.prop(p, "lock_pos")
        col = box.column(align=True)
        col.operator("arcam.recenter", icon=_icon("PIVOT_CURSOR"))
        col.operator("arcam.set_start", icon=_icon("OBJECT_ORIGIN"))

        box = L.box()
        box.label(text="Recording")
        box.prop(p, "output_dir", text="")
        box.prop(p, "round_frames")
        box.prop(p, "take_smooth")
        if st.recording:
            box.label(text=f"REC  {len(st.samples)} samples  {time.time() - st.rec_wall0:.1f}s", icon=_icon("REC"))
            box.operator("arcam.rec_stop", icon=_icon("SNAP_FACE"))
        else:
            box.operator("arcam.rec_start", icon=_icon("REC"))
        box.operator("arcam.import_csv", icon=_icon("IMPORT"))
        if p.camera is not None and p.camera.parent is not None:
            L.label(text="Camera is parented: keys are converted", icon=_icon("INFO"))
            L.label(text="to the parent's space (constraints ignored)")
        if st.last_msg:
            col = L.column(align=True)
            for line in textwrap.wrap(st.last_msg, 40)[:6]:
                col.label(text=line)
        if st.error:
            L.label(text="Error (see console): " + st.error[:60], icon=_icon("ERROR"))


# --------------------------------------------------------------------------
# Registration
# --------------------------------------------------------------------------
@bpy.app.handlers.persistent
def _on_load_pre(*_args):
    """A new .blend is being opened: save any running take, close the port, drop the timer."""
    _emergency_save()
    _stop_listener()
    STATE.timer_on = False


_classes = (
    ARCamProps,
    ARCAM_OT_start, ARCAM_OT_stop, ARCAM_OT_recenter, ARCAM_OT_set_start,
    ARCAM_OT_refresh_ip, ARCAM_OT_rec_start, ARCAM_OT_rec_stop, ARCAM_OT_import_csv,
    ARCAM_PT_main,
)


def _handler_lists():
    """load_pre always; exit_pre exists from Blender 5.1 (saves a take if Blender is closed while recording)."""
    lists = [bpy.app.handlers.load_pre]
    exit_pre = getattr(bpy.app.handlers, "exit_pre", None)
    if exit_pre is not None:
        lists.append(exit_pre)
    return lists


def register():
    for c in _classes:
        bpy.utils.register_class(c)
    bpy.types.Scene.arcam = PointerProperty(type=ARCamProps)
    for handlers in _handler_lists():
        if _on_load_pre not in handlers:
            handlers.append(_on_load_pre)


def unregister():
    _emergency_save()
    _stop_listener()
    try:
        if bpy.app.timers.is_registered(_tick):
            bpy.app.timers.unregister(_tick)
    except Exception:
        pass
    STATE.timer_on = False
    for handlers in _handler_lists():
        if _on_load_pre in handlers:
            handlers.remove(_on_load_pre)
    del bpy.types.Scene.arcam
    for c in reversed(_classes):
        bpy.utils.unregister_class(c)


if __name__ == "__main__":
    register()
