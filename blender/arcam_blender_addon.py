bl_info = {
    "name": "ARKit Virtual Camera (UDP)",
    "author": "previz helper",
    "version": (1, 0, 0),
    "blender": (3, 0, 0),
    "location": "3D Viewport > Sidebar (N) > ARCam",
    "description": "Drive a Blender camera live from an iPhone (ARKit pose over UDP) and record takes",
    "category": "Animation",
}

import bpy
import csv
import math
import os
import socket
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
# Shared state (the network thread only touches `latest`, `counter`, etc.
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
        self.applied = 0
        self.last_rx = 0.0
        self.last_ts = None
        self.trk = 0
        self.ref = None
        self.base = Matrix.Identity(4)
        self.recenter = False
        self.sm_loc = None
        self.sm_rot = None
        self.recording = False
        self.samples = []
        self.rec_t0 = None
        self.rec_start_frame = 1
        self.scene_name = ""
        self.last_redraw = 0.0
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
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-6 or not all(math.isfinite(v) for v in (px, py, pz, n, ts)):
        return None
    return seq, ts, (px, py, pz), (qw / n, qx / n, qy / n, qz / n), trk


def _listen(sock, stop_evt):
    st = STATE
    while not stop_evt.is_set():
        try:
            data, _addr = sock.recvfrom(2048)
        except socket.timeout:
            continue
        except ConnectionResetError:
            continue  # Windows can raise WSAECONNRESET (10054) on UDP sockets: not fatal
        except OSError:
            break
        pkt = _parse(data)
        if pkt is None:
            continue
        _seq, ts, pos, quat, trk = pkt
        with st.lock:
            # drop old/out-of-order UDP packets (accept a big backwards jump = phone restarted)
            if st.last_ts is not None and ts <= st.last_ts and ts > st.last_ts - 10.0:
                continue
            st.last_ts = ts
            st.latest = (ts, pos, quat, trk)
            st.trk = trk
            st.counter += 1
            st.last_rx = time.time()


def _local_ips():
    ips = set()
    try:
        for ip in socket.gethostbyname_ex(socket.gethostname())[2]:
            if not ip.startswith("127."):
                ips.add(ip)
    except OSError:
        pass
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))  # no packet is actually sent
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return sorted(ips)


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


def _apply(st, p, cam, pkt):
    ts, pos, quat, trk = pkt
    if trk == 0:  # ARKit says tracking not available: ignore junk pose
        return
    M = _arkit_to_blender(pos, quat, p.pos_scale, p.lock_pos, p.roll_offset)
    if st.ref is None or st.recenter:
        st.ref = st.base @ M.inverted()
        st.recenter = False
        st.sm_loc = None
    target = st.ref @ M
    loc, rot, _s = target.decompose()

    if st.recording:
        if st.rec_t0 is None:
            st.rec_t0 = ts
        st.samples.append((ts - st.rec_t0, tuple(loc), (rot.w, rot.x, rot.y, rot.z), trk))

    a = 1.0 - p.smoothing
    if st.sm_loc is None or a >= 0.999:
        sl, sr = loc.copy(), rot.copy()
    else:
        sl = st.sm_loc.lerp(loc, a)
        sr = st.sm_rot.slerp(rot, a)
    st.sm_loc, st.sm_rot = sl.copy(), sr.copy()
    cam.matrix_world = Matrix.Translation(sl) @ sr.to_matrix().to_4x4()


def _redraw():
    try:
        for w in bpy.context.window_manager.windows:
            for a in w.screen.areas:
                if a.type == "VIEW_3D":
                    a.tag_redraw()
    except Exception:
        pass


def _tick():
    st = STATE
    if not st.running:
        st.timer_on = False
        return None
    try:
        scn = bpy.data.scenes.get(st.scene_name)
        if scn is not None:
            p = scn.arcam
            cam = p.camera
            with st.lock:
                latest, counter = st.latest, st.counter
            if cam is not None and latest is not None and counter != st.applied:
                st.applied = counter
                _apply(st, p, cam, latest)
                _redraw()
            elif time.time() - st.last_redraw > 0.25:
                _redraw()
            if time.time() - st.last_redraw > 0.25:
                st.last_redraw = time.time()
    except Exception as e:  # never let the timer die silently
        st.error = str(e)
        traceback.print_exc()
    return 1.0 / 60.0


# --------------------------------------------------------------------------
# Recording helpers
# --------------------------------------------------------------------------
def _out_dir(p):
    d = p.output_dir or ""
    if d.startswith("//") and not bpy.data.filepath:
        d = ""
    d = bpy.path.abspath(d) if d else ""
    if not d or not os.path.isabs(d):
        d = os.path.join(os.path.expanduser("~"), "ARCam_takes")
    os.makedirs(d, exist_ok=True)
    return d


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


def _bake(scn, cam, samples, start_frame, name, round_frames):
    """Keyframe the samples on `cam` in a NEW action (previous takes are kept)."""
    fps = scn.render.fps / scn.render.fps_base
    ad = cam.animation_data_create()
    act = bpy.data.actions.new(name)
    act.use_fake_user = True
    ad.action = act
    cam.rotation_mode = "XYZ"

    keyed = {}
    prev = None
    for dt, loc, q, _trk in samples:
        f = start_frame + dt * fps
        if round_frames:
            f = float(round(f))
        quat = Quaternion(q)
        eul = quat.to_euler("XYZ", prev) if prev is not None else quat.to_euler("XYZ")
        prev = eul
        keyed[f] = (loc, eul)

    frames = sorted(keyed)
    for f in frames:
        loc, eul = keyed[f]
        cam.location = loc
        cam.rotation_euler = eul
        cam.keyframe_insert("location", frame=f)
        cam.keyframe_insert("rotation_euler", frame=f)
    if frames:
        scn.frame_end = max(scn.frame_end, int(round(frames[-1])))
        scn.frame_set(int(start_frame))
    return len(frames)


# --------------------------------------------------------------------------
# Properties
# --------------------------------------------------------------------------
def _is_camera(self, obj):
    return obj.type == "CAMERA"


class ARCamProps(bpy.types.PropertyGroup):
    camera: PointerProperty(name="Camera", type=bpy.types.Object, poll=_is_camera)
    port: IntProperty(name="UDP port", default=9000, min=1024, max=65535)
    pos_scale: FloatProperty(
        name="Move scale", default=1.0, min=0.001, soft_max=100.0,
        description="1 = real metres. >1 makes a small walk cover a bigger set (recenter after changing)")
    lock_pos: BoolProperty(name="Rotation only", default=False,
                           description="Ignore position, use only phone rotation (tripod/pan-tilt mode)")
    smoothing: FloatProperty(name="Smoothing", default=0.15, min=0.0, max=0.95,
                             description="Live smoothing of the viewport camera (recording stays raw)")
    roll_offset: FloatProperty(name="Roll offset", default=0.0, subtype="ANGLE",
                               min=-math.pi, max=math.pi,
                               description="Fine-tune if the view is rolled (try +/-90)")
    output_dir: StringProperty(name="Take folder", subtype="DIR_PATH", default="//ARCam_takes")
    round_frames: BoolProperty(name="Key whole frames", default=True,
                               description="One key per scene frame (clean). Off = keep sub-frame keys")


# --------------------------------------------------------------------------
# Operators
# --------------------------------------------------------------------------
class ARCAM_OT_start(bpy.types.Operator):
    bl_idname = "arcam.start"
    bl_label = "Start listening"

    def execute(self, context):
        st = STATE
        p = context.scene.arcam
        if p.camera is None:
            self.report({"ERROR"}, "Pick a camera first")
            return {"CANCELLED"}
        if st.running:
            return {"FINISHED"}
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind(("0.0.0.0", p.port))
        except OSError as e:
            self.report({"ERROR"}, f"Cannot open UDP port {p.port}: {e}")
            return {"CANCELLED"}
        s.settimeout(0.25)
        st.reset()
        st.sock = s
        st.scene_name = context.scene.name
        st.base = _cam_base(p.camera)
        st.ips = _local_ips()
        st.stop_evt = threading.Event()
        st.thread = threading.Thread(target=_listen, args=(s, st.stop_evt), daemon=True)
        st.thread.start()
        st.running = True
        if not st.timer_on:
            st.timer_on = True
            bpy.app.timers.register(_tick, first_interval=0.0)
        context.scene.camera = p.camera
        try:
            context.area.spaces.active.region_3d.view_perspective = "CAMERA"
        except Exception:
            pass
        return {"FINISHED"}


class ARCAM_OT_stop(bpy.types.Operator):
    bl_idname = "arcam.stop"
    bl_label = "Stop"

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

    def execute(self, context):
        STATE.ips = _local_ips()
        return {"FINISHED"}


class ARCAM_OT_rec_start(bpy.types.Operator):
    bl_idname = "arcam.rec_start"
    bl_label = "Record take"

    def execute(self, context):
        st = STATE
        p = context.scene.arcam
        if not st.running:
            self.report({"ERROR"}, "Start listening first")
            return {"CANCELLED"}
        if p.camera is not None and p.camera.parent is not None:
            self.report({"WARNING"}, "Camera has a parent: keyframes would be local. Unparent for best results")
        st.samples = []
        st.rec_t0 = None
        st.rec_start_frame = context.scene.frame_current
        st.recording = True
        return {"FINISHED"}


class ARCAM_OT_rec_stop(bpy.types.Operator):
    bl_idname = "arcam.rec_stop"
    bl_label = "Stop & save take"

    def execute(self, context):
        st = STATE
        scn = context.scene
        p = scn.arcam
        if not st.recording:
            return {"CANCELLED"}
        st.recording = False
        samples, st.samples = st.samples, []
        if len(samples) < 2 or p.camera is None:
            self.report({"WARNING"}, "Nothing recorded (no data received?)")
            return {"CANCELLED"}
        fps = scn.render.fps / scn.render.fps_base
        name = datetime.now().strftime("take_%Y%m%d_%H%M%S")
        msg = ""
        try:
            path = os.path.join(_out_dir(p), name + ".csv")
            _write_csv(path, samples, fps, st.rec_start_frame)
            msg = f" CSV: {path}"
        except OSError as e:
            self.report({"WARNING"}, f"Could not write CSV: {e}")
        n = _bake(scn, p.camera, samples, st.rec_start_frame, name, p.round_frames)
        self.report({"INFO"}, f"{name}: {len(samples)} samples -> {n} keys.{msg}")
        return {"FINISHED"}


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
        samples = []
        try:
            with open(self.filepath, newline="") as f:
                for r in csv.DictReader(f):
                    samples.append((float(r["time_s"]),
                                    (float(r["px"]), float(r["py"]), float(r["pz"])),
                                    (float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"])),
                                    int(r["tracking"])))
        except (OSError, KeyError, ValueError) as e:
            self.report({"ERROR"}, f"Bad CSV: {e}")
            return {"CANCELLED"}
        if len(samples) < 2:
            self.report({"ERROR"}, "CSV has no samples")
            return {"CANCELLED"}
        name = "import_" + os.path.splitext(os.path.basename(self.filepath))[0]
        n = _bake(scn, p.camera, samples, scn.frame_current, name, p.round_frames)
        self.report({"INFO"}, f"Imported {len(samples)} samples -> {n} keys")
        return {"FINISHED"}


# --------------------------------------------------------------------------
# UI
# --------------------------------------------------------------------------
class ARCAM_PT_main(bpy.types.Panel):
    bl_label = "ARKit Virtual Camera"
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
        box.prop(p, "port")
        if st.running:
            box.operator("arcam.stop", icon="PAUSE")
        else:
            box.operator("arcam.start", icon="PLAY")

        if st.running:
            fresh = st.last_rx and (time.time() - st.last_rx) < 1.0
            if fresh:
                names = {0: "not available", 1: "limited", 2: "normal"}
                L.label(text="Receiving - tracking " + names.get(st.trk, "?"),
                        icon="CHECKMARK" if st.trk == 2 else "ERROR")
            else:
                L.label(text="Listening... no data", icon="TIME")
        row = L.row()
        row.label(text="PC IP: " + (", ".join(st.ips) if st.ips else "?"))
        row.operator("arcam.refresh_ip", text="", icon="FILE_REFRESH")

        col = L.column(align=True)
        col.prop(p, "pos_scale")
        col.prop(p, "smoothing")
        col.prop(p, "roll_offset")
        col.prop(p, "lock_pos")
        col = L.column(align=True)
        col.operator("arcam.recenter", icon="PIVOT_CURSOR")
        col.operator("arcam.set_start", icon="OBJECT_ORIGIN")

        box = L.box()
        box.label(text="Recording")
        box.prop(p, "output_dir", text="")
        box.prop(p, "round_frames")
        if st.recording:
            box.label(text=f"REC  {len(st.samples)} samples", icon="REC")
            box.operator("arcam.rec_stop", icon="SNAP_FACE")
        else:
            box.operator("arcam.rec_start", icon="REC")
        box.operator("arcam.import_csv", icon="IMPORT")


_classes = (
    ARCamProps,
    ARCAM_OT_start, ARCAM_OT_stop, ARCAM_OT_recenter, ARCAM_OT_set_start,
    ARCAM_OT_refresh_ip, ARCAM_OT_rec_start, ARCAM_OT_rec_stop, ARCAM_OT_import_csv,
    ARCAM_PT_main,
)


def register():
    for c in _classes:
        bpy.utils.register_class(c)
    bpy.types.Scene.arcam = PointerProperty(type=ARCamProps)


def unregister():
    _stop_listener()
    for c in reversed(_classes):
        bpy.utils.unregister_class(c)
    del bpy.types.Scene.arcam


if __name__ == "__main__":
    register()
