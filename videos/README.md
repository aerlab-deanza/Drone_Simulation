# Videos

Everything needed to produce the project's videos.

| Path | What it is |
|---|---|
| `manim/` | Manim sources of the four theory videos (`v1_rotations.py` … `v4_control.py`) |
| `manim/common.py` | Shared style, plus the timing hooks that let scenes stretch to fit their captions |
| `manim/project_data.py` | Simulation data the animations plot (computed with `drone_sim`, cached) |
| `captions/` | The on-screen explanation of every scene (edit these to change the text) |
| `narration/` | Optional spoken scripts, if you want to add a voice-over later |
| `build_videos.py` | Render → fit timing to captions (or audio) → assemble with the caption band |
| `contact_sheet.py` | Tiles frames of every scene into one image, for layout review |
| `recording/` | Command-line demo recordings and the 3D replay of the offline demo |
| `live_demo_shot_list.md` | How to record the live OpenCR demo (screen + camera) |

Generated files go to `build/` (renders, caches) and `output/` (final MP4s),
both ignored by git.

## Requirements

```powershell
python -m pip install -e ".[videos]"     # Manim Community
```

You also need a LaTeX distribution (MiKTeX or TeX Live) for the equations,
and `ffmpeg`/`ffprobe` on `PATH` for syncing and assembly.

On Windows, MiKTeX can fail with *"cannot retrieve attributes for the
directory …\WinGet\Links\…"*. `build_videos.py` removes that folder from
`PATH` for Manim automatically. If you run `manim` by hand, do the same.

## Producing a theory video

The videos explain themselves with **on-screen captions**: no narration is
needed.

```powershell
python videos/build_videos.py all v1_rotations --preview   # fast 480p check
python videos/build_videos.py all v1_rotations             # final 1080p30
```

The final file is `videos/output/v1_rotations.mp4`. Each frame shows the
animation on top and a caption band underneath, so captions never cover the
animation.

- **Edit the explanations** in `captions/<video>.md`: one `## Scene` heading
  per scene, one `- bullet` per caption card. Re-run `all` to rebuild.
- **Timing:** each card stays up for its reading time (about 14 characters
  per second, at least 2.2 s). A scene is stretched to the longer of its
  natural length and its total reading time. Animations change speed only
  modestly (×0.75-×1.35); the pauses absorb the rest.
- **Optional voice-over:** `narration/<video>.md` holds a spoken script. To
  add your voice to a scene, save a clip named after the scene (e.g.
  `videos/audio/v1_rotations/S01Intro.wav`) and re-run `all`. That clip then
  sets the scene's length, and the captions stay on screen.
- `--no-captions` on `assemble` produces a version without the caption band.

Lengths with captions: video 1 ≈ 4.3 min, video 2 ≈ 3.9 min, video 3
≈ 4.6 min, video 4 ≈ 5.0 min.

## Demo recordings

```powershell
python videos/recording/record_demos.py        # install / lqr command videos (rendered terminal)
python videos/recording/record_screen_demo.py  # static demo: real screen capture with window interaction
python videos/recording/render_replay.py       # 3D replay of the offline demo
```

They write to `docs/_static/videos/`.
