# ARCam – iPhone → Blender live virtual camera

Use an iPhone as a **6DoF virtual camera** for live previz in Blender: walk around a real space and the Blender
camera follows your position and rotation in real time. Takes can be **recorded** (CSV + keyframes) and reused later
for previz or as camera-move reference.

- **Free**: no paid apps, no paid Apple developer account.
- **No Mac / no Xcode needed**: the iPhone app is built in the cloud by GitHub Actions and installed from Windows (or macOS).
- **Windows, macOS and Linux** for the Blender add-on (pure Python, no dependencies).

> Status: community project, written for a school previz. Fill in the "Tested on" table below with your own results.

---

## How it works

```
 iPhone (ARKit: camera + gyro + accelerometer fused → 6DoF pose, ~60 Hz)
        │   UDP packets over Wi-Fi  "AR1,seq,ts,px,py,pz,qx,qy,qz,qw,tracking"
        ▼
 Blender add-on (listener thread → main-thread timer → camera.matrix_world)
        │
        ├─ live camera in the 3D viewport
        └─ record: CSV file + keyframes (new Action per take)
```

Gyro alone cannot give position (it drifts within seconds), so the app uses **ARKit** world tracking, which fuses the
camera image with the motion sensors. No LiDAR is required (an iPhone 15 works).

## Repository layout

| Path | Purpose |
|---|---|
| `blender/arcam_blender_addon.py` | The Blender add-on (single file) |
| `ios/` | iPhone app source (SwiftUI + ARKit) and `project.yml` (XcodeGen) |
| `.github/workflows/build.yml` | CI: tests, unsigned IPA build, GitHub Release |
| `tools/fake_phone.py` | Simulated phone for testing Blender without an iPhone |
| `tests/test_listener.py` | Automated tests of the network/protocol layer |

---

## Features

### iPhone app (`ios/`)
- ARKit world tracking, gravity-aligned (Y up), real-world metres.
- Streams the pose over UDP at the camera frame rate (~60 Hz).
- Orientation picker: Portrait / Landscape L / Landscape R (the pose is sent already rotated for that screen orientation).
- Sends ARKit tracking state (none / limited / normal) with every packet.
- **Reset tracking** button; IP and port are remembered between launches; screen stays awake while streaming.

### Blender add-on (`blender/`)
- Sidebar panel **N → ARCam**; works on Blender 3.0+ (single `.py`, Install from Disk).
- **Start / Stop listening** on a chosen UDP port; shows the PC's IP address(es) to type in the app and the live
  connection / tracking status.
- Drives any chosen camera live; also sets it as the scene camera and switches the viewport to camera view.
- **Recenter to start**: snaps the camera back to its start pose using the phone's current pose as reference.
- **Set start = camera now**: use the camera's current pose as the new start point without a jump.
- **Move scale**: 1 = real metres; >1 lets a small walk cover a large virtual set.
- **Rotation only** mode (tripod / pan-tilt use).
- **Smoothing** of the live viewport camera (recording stays raw).
- **Roll offset** to fine-tune a view rolled by 90°.
- **Record take**: stores raw samples with the phone's own timestamps; on stop it writes a **CSV** and bakes **keyframes**
  into a **new Action** (earlier takes are kept with a fake user). Option: key whole frames or keep sub-frame keys.
- **Import take (CSV)**: re-bake any CSV take onto the camera at the current frame (also at another scene fps).
- Robustness: invalid / NaN / zero-quaternion packets ignored, out-of-order UDP dropped, port errors reported
  immediately, Windows UDP `WSAECONNRESET` handled, network thread never touches `bpy`, timer errors do not kill the stream.
- ARKit "tracking not available" frames are ignored rather than applied.

### Coordinate conversion
ARKit world (X right, Y up, Z toward viewer) → Blender world (X right, Y forward, Z up) is a +90° rotation about X.
Camera local axes are identical in both (look down −Z, up +Y), so only the world is converted.

### CSV format
`time_s, frame, tracking, px, py, pz, qw, qx, qy, qz, rx_deg, ry_deg, rz_deg`
Position in Blender world metres (after recenter and move scale), rotation as quaternion and as continuous XYZ Euler
degrees. `time_s` is in seconds from the first sample, so it is independent of the scene fps.

---

## Workflow
A. Every week due to apple 7days Id limit
1. Download the IPA (artifact, or Release asset after `git tag v1.0.0 && git push --tags`).
2. Install it from **Windows or macOS** with **Sideloadly** (free Apple ID; on Windows install iTunes and iCloud from
   Apple's website, not the Microsoft Store). The app is re-signed on your PC, so Xcode is never needed.
3. On the iPhone: enable **Developer Mode** (Settings → Privacy & Security), then trust your Apple ID profile
   (Settings → General → VPN & Device Management).
4. In Blender: **Preferences → Add-ons → Install from Disk →** `arcam_blender_addon.py`, enable it.

B.
1. PC and iPhone on the same network (own router or iPhone hotspot; many school/guest Wi-Fi networks block
   device-to-device traffic). Allow Blender through the Windows firewall on **private** networks.
2. Blender: pick the camera, press **Start listening**, note the PC IP shown in the panel.
3. iPhone app: enter that IP and port (default 9000), press **Start streaming**, accept the camera and
   local-network permissions the first time.
4. Hold the phone where you want the shot to start, press **Recenter to start**. Walk.
5. **Record take** → shoot → **Stop & save take**. The CSV path is reported in Blender's status bar.
6. Review: press **Stop** (listening) before scrubbing, otherwise live data overrides the animation.

---

## Packet protocol (UDP, ASCII, one packet per sample)
`AR1,<seq>,<timestamp_s>,<px>,<py>,<pz>,<qx>,<qy>,<qz>,<qw>,<tracking>` – ARKit world space, camera-to-world pose
for the selected screen orientation; `tracking` 0 = not available, 1 = limited, 2 = normal.

## Limitations
- ARKit drifts over long takes and tracks poorly in dark, blank, reflective or very fast-moving scenes.
- Good for previz and as camera-move reference; not a replacement for proper match-moving on difficult shots.
- The phone's pose is not the pose of a cinema camera: offset and lens must be matched manually.
- UDP is unencrypted; use a trusted network.

## Troubleshooting
| Symptom | Fix |
|---|---|
| "Listening… no data" | Same network? Firewall? Correct IP (use the one on the same subnet as the phone)? |
| Cannot open UDP port | Another program uses the port; change it in both Blender and the app |
| View rolled 90° | Change the orientation picker in the app, or the Roll offset in Blender |
| Camera jumps when pressing Recenter | Expected: it returns to the start pose. Use *Set start = camera now* to keep the current one |
| Movement scale feels wrong | Change *Move scale*, then Recenter |
| App stops working after a week | Free Apple ID expiry: re-install with Sideloadly |
| CI iOS job fails | Open the failing step's log in the Actions tab and open an issue with it |

## Tested on
| Blender | OS | iPhone / iOS | Result |
|---|---|---|---|
| _Ok_ | _5.3_ | _15/27.0.1_ | _Perfect_ |

## Contributing / roadmap ideas
Lens/FOV sync from ARKit intrinsics, OSC output, multi-take manager, Android (ARCore) client, in-app record button.
