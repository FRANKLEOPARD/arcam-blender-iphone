# Changelog

## 2.0.0

### Bugs fixed
**Blender add-on**
- **Takes were keyed up to half a frame late.** Several phone samples fell on the same frame and the last one won.
  Keys are now interpolated at the exact time of each frame.
- **Parented cameras got wrong keyframes** (world values written into local properties). Keys are now converted to
  the parent's space.
- **The camera snapped back to the previous take** whenever the frame changed while live. Starting to listen now
  unassigns the camera's action (kept with a fake user, never deleted) and says so in the panel.
- **The camera jumped when Move scale, Rotation only or Roll offset changed** while live. It now re-anchors smoothly.
- **After opening another .blend while listening**, Blender's timer was silently removed but the add-on still believed
  it was running, so nothing moved and Start did nothing. A file-load handler now stops cleanly. Timer registration
  checks `bpy.app.timers.is_registered` instead of a flag that could go stale.
- **A running take was lost** when opening another file or disabling the add-on (and, from Blender 5.1, quitting).
  It is now saved to `~/ARCam_takes/…_unsaved.csv`.
- **Start could freeze Blender for several seconds on some Macs** (hostname lookup). IP detection is now time-limited
  and lists the most likely address first, without link-local addresses.
- **Renaming the scene while listening stopped everything silently.** The add-on now follows the active scene.
- Importing a CSV while listening was immediately overwritten by the live phone. Import now asks you to stop first.
- The UDP port field can no longer be edited while listening (it had no effect).
- Clear error hints for "port reserved by Windows" (WinError 10013) and "port already in use".
- An unknown icon name in a future Blender version can no longer break the whole panel.
- Packets with an invalid tracking value are rejected.
- Long status messages wrap instead of being cut off.
- The action is created by `keyframe_insert()` itself, which is the robust path for every Blender version including
  the slotted Actions introduced in Blender 4.4; each take's action is named like its CSV file.

**iPhone app**
- **"Reset tracking" made the Blender camera jump** (ARKit moves its world origin). The app now sends a recenter
  automatically on the first pose of the new session (stale frames from before the reset are ignored).
- **The connection never recovered after a network failure** (e.g. Wi-Fi drop, app in background). It now reconnects
  by itself.
- **The known iOS "Network is down" Local Network permission problem** now shows what to do (enable Local Network for
  ARCam; restart the iPhone if it is already on).
- The landscape choices were ambiguous ("Landscape L/R"). They now say where the top of the phone points.
- Replaced deprecated SwiftUI APIs; added an app icon and the app version in Settings.
- The generated `Info.plist` now lives outside the source folder (avoids Xcode's "Multiple commands produce
  Info.plist" error).

**Build (GitHub Actions)**
- **GitHub removed Node 20 from Actions in September 2026.** The workflow now uses the Node 24 versions:
  `actions/checkout@v5`, `actions/setup-python@v6`, `actions/upload-artifact@v6`, `actions/download-artifact@v7`,
  `softprops/action-gh-release@v3`.
- The workflow prints the Xcode version (the runner's default Xcode changes over time), fails if the IPA is missing,
  and keeps symbolic links when zipping the app.

**Tools**
- `fake_phone.py` no longer crashes on Windows-specific UDP receive errors.

### Tests
36 automated tests (was 24), including exact frame timing, parented cameras, the Start/Stop operators, CSV import,
file-load safety, handler registration and a full phone ↔ Blender loop over real UDP. A mutation check (deliberately
breaking 12 fixes one by one) confirmed each break is caught by at least one test.

## 1.1.0
Adaptive live smoothing (One Euro filter), zero-lag take smoothing, record/recenter from the phone, status sent back
to the phone, redesigned one-screen iPhone app.

## 1.0.0
First version: live ARKit camera over UDP, recording to CSV + keyframes, CSV import, Windows support.
