# ARCam – iPhone → Blender live virtual camera

Use an iPhone as a **6DoF virtual camera** for live previz in Blender: walk around a real space and the Blender
camera follows your position and rotation in real time. Record takes from the phone (CSV + keyframes) and reuse them
later for previz or as camera-move reference.

- **Free**: no paid apps, no paid Apple developer account.
- **No Xcode needed**: download the ready-made iPhone app and install it with Sideloadly
  (from Windows or macOS, including old Macs).
- **Windows, macOS and Linux** for the Blender add-on (pure Python, no dependencies).

> Status: v2.0.0, written for a school previz. The add-on is covered by 36 automated tests (run outside Blender with
> stand-ins for Blender's API) and the iPhone app is compiled by CI; both still need to be tried for real. Please fill
> in the "Tested on" table below with your own results.

---

## How it works

```
 iPhone (ARKit: camera + gyro + accelerometer fused → 6DoF pose, 60 Hz)
        │  UDP over Wi-Fi: AR1,seq,ts,px,py,pz,qx,qy,qz,qw,tracking
        │  UDP commands  : CMD,recenter | CMD,rec_start | CMD,rec_stop
        ▼
 Blender add-on  (listener thread → 60 Hz timer → adaptive filter → camera.matrix_world)
        │
        ├─ live camera in the 3D viewport
        ├─ record: raw CSV + keyframes (new Action per take, exact frame timing, optional zero-lag smoothing)
        └─ UDP status back to the phone: ST,recording,samples,seconds,takes
```

Gyro alone cannot give position (it drifts within seconds), so the app uses **ARKit** world tracking, which fuses the
camera image with the motion sensors. No LiDAR is required (an iPhone 15 works).

## Repository layout

| Path | Purpose |
|---|---|
| `blender/arcam_blender_addon.py` | The Blender add-on (single file) |
| `ios/` | iPhone app: SwiftUI + ARKit source, app icon, `project.yml` (XcodeGen) |
| `.github/workflows/build.yml` | CI: tests, unsigned IPA build, GitHub Release |
| `tools/fake_phone.py` | Simulated phone for testing Blender without an iPhone |
| `tests/test_listener.py` | Automated tests (network, filters, camera pipeline, recording, operators, phone↔Blender loop) |
| `tests/fake_mathutils.py` | numpy stand-in for Blender's `mathutils`, used only by the tests |
| `CHANGELOG.md` | What changed in each version, including every bug fixed |
| `BUILDING.md` | For maintainers: building the app with GitHub Actions, publishing a release, running the tests |

---

## Features

### iPhone app (`ios/`)
Simple on purpose: one screen, one big button.
- **Status card** in plain words: tracking quality (green / orange / red) with the reason and what to do
  ("Move more slowly", "Point at a textured, well-lit area"…), and whether **Blender is connected** with the live
  packet rate in Hz. Haptic tap when tracking gets worse.
- **Record button** (like the Camera app) with a running timer and sample counter. It controls the recording in Blender,
  so you can shoot with the phone in your hands and never touch the computer. The phone only shows "recording" once
  Blender confirms it, and shows how many takes are saved.
- **Recenter** (camera returns to its start pose) and **Reset tracking** (restarts ARKit, then re-anchors Blender
  automatically so the camera does not jump).
- **Settings** (gear): computer IP, port, how you hold the phone (Portrait / Top right / Top left). Opens by itself
  on the first launch; values are remembered.
- Reconnects by itself if the network connection fails; explains the known iOS "Local Network" permission problem when
  it happens; warns when the phone overheats; keeps the screen awake while streaming; ARKit runs at 60 fps with unneeded
  features (plane detection, light estimation, environment textures) switched off to save CPU and heat.

### Blender add-on (`blender/`)
- Sidebar panel **N → ARCam**; Blender 3.0 and newer, including 4.x and 5.x (single `.py`, Install from Disk).
- **Start / Stop listening** on a UDP port; shows the PC's IP address(es), the most likely one first, and the live
  connection / tracking status. Clear messages for "port already used" and "port reserved by Windows".
- Drives any chosen camera live; sets it as the scene camera and switches the viewport to camera view. If the camera
  already has an animation, it is unassigned (kept with a fake user, never deleted) so it cannot fight the phone.
- **Camera feel**: live smoothing, move scale, roll offset, rotation-only mode, Recenter, Set start = camera now.
  Changing move scale, rotation-only or roll while live re-anchors the camera, so it never jumps.
- **Record take** (from Blender or from the phone): raw samples with the phone's own timestamps. On stop it writes a
  **CSV** and bakes **keyframes** into a **new Action** named like the CSV (earlier takes are kept).
  Keys are **interpolated at the exact time of each frame** (no half-frame timing error). Parented cameras are
  supported (keys are converted to the parent's space).
- **Import take (CSV)**: re-bake any CSV take onto the camera at the current frame (also at another scene fps).
- **A take is never lost**: if you open another .blend, disable the add-on, or (Blender 5.1+) quit while recording,
  the take is saved to `~/ARCam_takes/…_unsaved.csv`.

### Smoothing
| Where | What | Setting |
|---|---|---|
| Live view | **One Euro filter** (speed-adaptive low-pass) on position and rotation: strong smoothing when the phone is nearly still (removes jitter), almost none when it moves fast (little lag). Snaps instead of smearing on ARKit relocalization jumps or after a gap in the data. | **Live smoothing** 0–1 (default 0.5, 0 = off) |
| Recorded take | **Zero-lag Gaussian smoothing** of the keyframes, done offline when the take is baked. The CSV always keeps the **raw** data, and Import re-bakes with the current value. | **Take smoothing (s)** (default 0.03, 0 = off) |

Measured in the simulation tests: 2 mm of jitter on a still phone drops to about 0.8 mm at the default live setting,
with about 3 cm of lag while walking at 1 m/s. Take smoothing at 0.05 s reduces jitter about 3× with zero lag.
Real results depend on the scene: tune the two sliders on set.

**Getting the best raw tracking** (smoothing cannot fix bad tracking): well-lit, textured surroundings (avoid blank
walls, mirrors, glass); move steadily, no sudden whips; do not cover the lens; press *Reset tracking* if it drifts;
keep the phone cool; use your own router or hotspot (the app's Hz readout should stay near 60).

### Coordinate conversion
ARKit world (X right, Y up, Z toward viewer) → Blender world (X right, Y forward, Z up) is a +90° rotation about X.
The camera starts at its own pose when you press *Recenter*; the phone's later movement is applied relative to that.

### CSV format
`time_s, frame, tracking, px, py, pz, qw, qx, qy, qz, rx_deg, ry_deg, rz_deg`
Comma-separated, dot decimals. Position in Blender world metres (after recenter and move scale), rotation as a
quaternion and as continuous XYZ Euler degrees. `time_s` is seconds from the first sample, independent of the scene fps.

---

## Getting started

### What you need
- An **iPhone** on iOS 16 or newer (iPhone 8 or later; no LiDAR needed).
- A **Windows or Mac computer** with **Blender 3.0 or newer**.
- A **USB cable** for the iPhone (only to install the app).
- A free **Apple ID** (a second one just for this is a good idea).
- Both devices on the **same Wi-Fi**, or the computer joined to the iPhone's Personal Hotspot.
  School or guest Wi-Fi often blocks devices from talking to each other.

### 1. Download
Open the **Releases** page of this repository (right side of the GitHub page) and download from the latest release:
- `arcam_blender_addon.py` (the Blender add-on)
- `ARCamStream.ipa` (the iPhone app)

### 2. Install the Blender add-on
1. In Blender: **Edit → Preferences → Add-ons**.
2. Click the arrow menu (top right) → **Install from Disk…** and pick `arcam_blender_addon.py`.
3. Tick the add-on to enable it. The **ARCam** tab appears in the 3D Viewport sidebar (press **N**).

If you are updating from an older version, install the new file the same way and restart Blender. The add-on and the
iPhone app must be the same version.

### 3. Install the iPhone app (with Sideloadly)
1. Install **Sideloadly** on your computer.
   On Windows, also install **iTunes** and **iCloud** from Apple's website (not the Microsoft Store versions,
   otherwise the iPhone is not detected).
2. Plug in the iPhone, unlock it, and tap **Trust** if asked.
3. In Sideloadly, drag `ARCamStream.ipa` into the window, enter your Apple ID, and click **Start**.
4. On the iPhone:
   - **Settings → Privacy & Security → Developer Mode** → turn it on (the phone restarts).
   - **Settings → General → VPN & Device Management** → tap your Apple ID → **Trust**.

With a free Apple ID the app stops opening after **7 days**: plug in the phone and install the same `.ipa` again with
Sideloadly. A free Apple ID allows 3 sideloaded apps at a time.

### 4. First connection (once)
1. In Blender, open the **ARCam** tab, pick your camera, and press **Start listening**.
   Allow Blender through the firewall when asked:
   - Windows: tick **Private** and **Public** networks (a hotspot counts as Public).
   - Mac: click **Allow**.
2. Note the **PC IP** shown in the panel.
3. On the iPhone, open **ARCam**. Settings opens by itself: type that IP (port 9000), and choose how you hold the phone.
4. Tap **Start streaming** and allow **Camera** and **Local Network** access.

You are connected when the app shows **two green dots**: *Tracking good* and *Blender connected*.

### 5. Each session
1. Blender: **Start listening**. iPhone: **Start streaming**.
2. Hold the phone where the shot starts and tap **Recenter**. Walk: the Blender camera follows.
3. Tap the **red button** to record, tap again to stop. The take is saved as a CSV file and as keyframes.
4. To review a take: press **Stop** (listening) in Blender, then play or scrub the timeline.
   Starting to listen again unassigns the take from the camera; it stays in the file and can be reassigned in the
   Action editor.
5. To reuse a take later, in another shot or at another frame rate: **Import take (CSV)**.

If something does not work, see **Troubleshooting** below.

---

## Network protocol (UDP, ASCII, one message per packet)
| Direction | Message | Meaning |
|---|---|---|
| phone → Blender | `AR1,<seq>,<timestamp_s>,<px>,<py>,<pz>,<qx>,<qy>,<qz>,<qw>,<tracking>` | ARKit world, camera-to-world pose for the selected screen orientation; `tracking` 0 = none, 1 = limited, 2 = normal |
| phone → Blender | `CMD,recenter` / `CMD,rec_start` / `CMD,rec_stop` | remote control (unknown commands are ignored) |
| Blender → phone | `ST,<recording 0/1>,<samples>,<seconds>,<takes saved>` | sent 4×/s to the phone's address; drives "Blender connected" |

UDP is unencrypted and unauthenticated: anyone on the same network could send these messages. Use a trusted network.

## Known limitations
- ARKit drifts over long takes and tracks poorly in dark, blank, reflective or very fast-moving scenes.
- Good for previz and as camera-move reference; not a replacement for proper match-moving on difficult shots.
- The phone's pose is not the pose of a cinema camera: offset and lens must be matched manually.
- Lens / field of view is not synced from the phone.
- Parented cameras: the parent is assumed static during the take and constraints are ignored.
- Baking very long takes (several minutes) can pause Blender for a few seconds.

## Troubleshooting (including known OS issues)
| Symptom | Fix |
|---|---|
| Phone shows "Waiting for Blender…" | Same network? Correct IP (same subnet as the phone)? Firewall (see Getting started, step 4)? iOS Settings → Privacy & Security → Local Network → ARCam on? |
| Phone says "iOS blocks local network access" (error "Network is down") | Turn ARCam on in Settings → Privacy & Security → Local Network. If it is already on, **restart the iPhone**: iOS sometimes does not pick up the change (a known iOS issue). |
| Blender receives data but the phone says "Waiting for Blender" | The PC answers from another network adapter: disable VPNs / virtual adapters (Hyper-V, WSL, VirtualBox) or use a simpler network. |
| Several IPs shown in Blender | Use the one on the same subnet as the phone (e.g. both 192.168.1.x, or 172.20.10.x on an iPhone hotspot). |
| "Cannot open UDP port … 10013" (Windows) | The port is reserved by Windows (Hyper-V/WSL): pick another port (e.g. 9500) in Blender and in the app. |
| "Cannot open UDP port … already used" | Another program or a second Blender window uses it: close it or change the port on both sides. |
| Windows works on home Wi-Fi but not on the iPhone hotspot | Windows treats the hotspot as a *Public* network: allow Blender on Public networks, or set the network to Private. |
| View rolled 90° or 180° | App Settings → "How you hold the phone" (Top right / Top left), or Roll offset in Blender. |
| Camera jumps when pressing Recenter | Expected: it returns to the start pose. Use *Set start = camera now* to keep the current one. |
| Still jittery | Raise *Live smoothing*; improve lighting / texture; check the Hz readout on the phone. |
| Feels laggy | Lower *Live smoothing*; check Wi-Fi (Hz readout should stay near 60). |
| Record button greyed out | Blender is not connected yet. |
| CSV opens in one column in Excel (French/European settings) | Excel expects `;` there. Use *Data → From Text/CSV* and choose comma as the separator. Blender's Import is unaffected. |
| App stops opening after a week | Free Apple ID expiry: re-install the IPA with Sideloadly. |
| Sideloadly does not see the iPhone (Windows) | Install iTunes and iCloud from Apple's website, not the Microsoft Store; unlock the phone and tap Trust. |
| "Untrusted Developer" when opening the app | Settings → General → VPN & Device Management → your Apple ID → Trust. |

## Tested on
| Blender | OS | iPhone / iOS | Result |
|---|---|---|---|
| _Blender_ | _5.3.0_ | _15/2.0.1_ | _Ok_ |

## Contributing / roadmap ideas
Lens/FOV sync from ARKit intrinsics, OSC output, multi-take manager, Android (ARCore) client.
