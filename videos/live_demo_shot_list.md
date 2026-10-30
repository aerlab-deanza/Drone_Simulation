# Live OpenCR demo: recording guide and shot list

Target: one 2-3 minute video, **screen + camera picture-in-picture**, uploaded
to YouTube and embedded on the docs page `docs/demos/live.md` (replace `TBD`
with the video ID). No narration is required; captions are listed per shot
if you want to add them in the editor. If you narrate, the "say" lines are a
script.

## Equipment

- OpenCR board with `firmware/opencr_imu_stream` flashed, USB cable.
- A webcam or phone on a small tripod, pointed at a clear table area. Mark the
  board's **nose (+X)** with a red sticker or tape: the on-screen model's
  nose is red.
- Plain, uncluttered desk; good front lighting (no backlight from a window).
- Keep the board away from laptops, speakers and steel desk legs, which
  disturb the magnetometer.

## OBS Studio setup (free, obsproject.com)

1. **Settings → Video:** Base and Output resolution 1920×1080, 30 fps.
2. **Settings → Output → Recording:** format **MP4** (or MKV, then
   *File → Remux*), encoder hardware (NVENC/QuickSync) if available,
   quality "High".
3. **Scene "Live demo":**
   - Source 1 - *Window Capture* of the Matplotlib window "Live Simulation".
     Right-click → *Transform → Fit to screen*.
   - Source 2 - *Video Capture Device* (webcam/phone). Resize to about
     30 % width and place it **bottom-right**, over the accelerometer chart's
     empty corner. Add a 4 px white border (Filters → *Image Mask/Blend* or
     a colour-source frame).
4. **Scene "Terminal":** *Window Capture* of PowerShell, for shot 1.
5. Use a hotkey for scene switching (Settings → Hotkeys).
6. Before recording: maximize the viewer window on a ≥1400 px wide screen
   so it uses the side-by-side layout.

## Shot list

| # | Duration | Scene | Action | On-screen check | Caption / say |
|---|---|---|---|---|---|
| 1 | 0:00-0:15 | Terminal | Type `python -m drone_sim.live --port COM5` and press Enter (use your port). | The four start-up lines are printed. | "Start the live viewer on the board's serial port." |
| 2 | 0:15-0:25 | Live demo | Board **flat and still** on the table. Don't touch it. | Status goes "calibrating 1/150 … calibrated". | "Keep it still: the viewer measures the gyro bias and checks that the accelerometer reads 1 g." |
| 3 | 0:25-0:30 | Live demo | Bump the table lightly *during* calibration (optional retake). | Status: "motion detected; hold still to calibrate". | "Any motion restarts calibration." |
| 4 | 0:30-0:50 | Live demo | Pick the board up. **Roll** slowly right ~45°, back, left ~45°, back. | Model rolls the same way; roll read-out follows. | "Roll." |
| 5 | 0:50-1:10 | Live demo | **Pitch** nose up ~30° and hold 3 s, then nose down and hold 3 s. | Model climbs while nose-up, descends nose-down (virtual flight). | "Pitch. The virtual drone flies along its nose, so nose-up climbs." |
| 6 | 1:10-1:30 | Live demo | **Yaw**: turn the board flat on the table 90° left, then back. | Yaw read-out changes ~90°; path turns. | "Yaw. Gravity can't measure heading, so this is the gyro and, if present, the magnetometer." |
| 7 | 1:30-1:50 | Live demo | Point the nose **straight up** (90° pitch), hold 3 s, slowly rotate it about its own axis, come back down. | Model stays smooth; no flipping or spinning. | "Straight up: Euler angles would hit gimbal lock here. The filter uses quaternions, so nothing breaks." |
| 8 | 1:50-2:10 | Live demo | Quick figure-eight wave with the board for 5 s, then put it flat on the table. | Roll/pitch return to about 0° once the board rests flat. | "While it moves, the accelerometer is ignored; once it is still, gravity pulls the estimate back." |
| 9 | 2:10-2:25 | Live demo | Leave it still. | Roll/pitch stay ~0°, yaw nearly constant; the virtual drone cruises level. | "At rest the accelerometer keeps roll and pitch locked to gravity; only yaw can slowly drift." |
| 10 | 2:25-2:30 | Live demo | Close the window. | - | End card: docs URL. |

## Retake tips

- Move **slowly** (about 2-3 s per gesture) so viewers can match the camera
  with the screen.
- Keep the board's red nose pointing roughly toward the screen's +X
  direction at start (the viewer starts at heading 0).
- If the status stays "invalid accelerometer scale", reflash the sketch in
  this repository.
- Close Arduino's Serial Monitor before starting; it holds the port.

## Editing and upload

1. Trim dead air at the start/end (any editor: Clipchamp, DaVinci Resolve).
2. Optional captions from the table above.
3. Export 1080p MP4, upload to YouTube (Unlisted or Public), copy the video ID
   (the part after `v=`).
4. In `docs/demos/live.md` replace ``{youtube} TBD`` with
   ``{youtube} <video-id>``. The docs build embeds it automatically.
