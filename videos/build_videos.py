"""Render, time and assemble the Manim theory videos (with on-screen captions).

Run from the repository root (needs ffmpeg/ffprobe on PATH and LaTeX)::

    python videos/build_videos.py all      v1_rotations           # full build → videos/output/v1_rotations.mp4
    python videos/build_videos.py all      v1_rotations --preview # same at 480p15, much faster

    python videos/build_videos.py render   v1_rotations [S05GimbalLock]   # silent scene previews (480p15)
    python videos/build_videos.py measure  v1_rotations           # record each scene's natural timing
    python videos/build_videos.py sync     v1_rotations [--preview]       # fit scenes to captions / audio, render
    python videos/build_videos.py assemble v1_rotations [--preview]       # join scenes + caption band (+ audio)

How timing works:

* Each scene's on-screen explanation lives in ``videos/captions/<video>.md``
  (one bullet per caption card). A card stays up for its reading time, about
  14 characters per second and at least 2.2 s.
* A scene is stretched to the longer of its natural length and its total
  reading time. Animations change speed only modestly (×0.75-1.35); the
  pauses absorb the rest.
* Optional narration: if ``videos/audio/<video>/<Scene>.wav`` (or ``.mp3``,
  ``.m4a``, ``.flac``) exists, the clip's length drives that scene instead,
  and the audio is mixed in. Captions are still shown.

The final frame is 16:9. The animation is scaled to 84 % at the top, and the
caption band runs underneath, so captions never cover the animation.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import textwrap
from pathlib import Path

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent
MANIM_DIR = ROOT / "manim"
CAPTIONS = ROOT / "captions"
BUILD = ROOT / "build"
MEDIA = BUILD / "media"
AUDIO = ROOT / "audio"
OUTPUT = ROOT / "output"
TIMINGS = BUILD / "timings.json"
NOMINAL = BUILD / "nominal.json"
STATS = BUILD / "scene_stats.json"   # written by the scenes themselves
AUDIO_EXTENSIONS = (".wav", ".mp3", ".m4a", ".flac")

TAIL_S = 0.4                 # silence kept after each narration clip
READING_CPS = 14.0           # caption reading speed (characters per second)
MIN_CARD_S = 2.2             # shortest time a caption card stays up
EDGE_S = 0.3                 # no caption in the first/last 0.3 s of a scene
SCENE_FRACTION = 0.84        # animation height as a fraction of the frame
BACKGROUND = "#0f1117"       # matches the Manim background
CAPTION_COLOR = (232, 232, 232)
RULE_COLOR = (42, 47, 58)
FONT_FILE = "C:/Windows/Fonts/segoeui.ttf"
FALLBACK_FONT_FILE = "C:/Windows/Fonts/seguisym.ttf"   # maths symbols such as ⊗

PLAY_SCALE_RANGE = (0.75, 1.35)   # animations change speed only modestly
MIN_WAIT_SCALE = 0.3


# -- scenes and rendering -------------------------------------------------------
def scene_names(video: str) -> list[str]:
    """Scene classes ``S01…`` of a video module, in file order."""
    source = (MANIM_DIR / f"{video}.py").read_text(encoding="utf-8")
    return re.findall(r"^class (S\d\d\w*)\(", source, flags=re.MULTILINE)


def clean_env() -> dict[str, str]:
    """Environment for Manim: drop WinGet's Links folder from PATH.

    MiKTeX aborts when it scans a broken entry in that folder. Manim encodes
    video through PyAV, so it does not need ffmpeg on PATH.
    """
    env = dict(os.environ)
    parts = env.get("PATH", "").split(os.pathsep)
    env["PATH"] = os.pathsep.join(p for p in parts if "WinGet\\Links" not in p and "WinGet/Links" not in p)
    env["MPLBACKEND"] = "Agg"
    return env


def quality_folder(hq: bool) -> str:
    return "1080p30" if hq else "480p15"


def scene_file(video: str, scene: str, hq: bool) -> Path:
    return MEDIA / "videos" / video / quality_folder(hq) / f"{scene}.mp4"


def render(video: str, scenes: list[str] | None = None, hq: bool = False) -> None:
    scenes = scenes or scene_names(video)
    # -qh alone means 1080p at 60 fps; force 30 fps to match quality_folder().
    quality = ["-qh", "--fps", "30"] if hq else ["-ql"]
    command = [sys.executable, "-m", "manim", *quality, "--media_dir", str(MEDIA), f"{video}.py", *scenes]
    print(f"Rendering {video}: {len(scenes)} scene(s) at {quality_folder(hq)}...")
    subprocess.run(command, cwd=MANIM_DIR, env=clean_env(), check=True)


def duration(path: Path) -> float:
    result = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of",
                             "csv=p=0", str(path)], capture_output=True, text=True, check=True)
    return float(result.stdout.strip())


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}


def save_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")


def measure(video: str) -> None:
    """Render at natural speed (preview quality); scenes record their play/wait totals."""
    timings = load_json(TIMINGS)
    for scene in scene_names(video):
        timings.pop(f"{video}.{scene}", None)
    save_json(TIMINGS, timings)
    render(video, hq=False)
    nominal = load_json(NOMINAL)
    for scene in scene_names(video):
        nominal[f"{video}.{scene}"] = duration(scene_file(video, scene, hq=False))
    save_json(NOMINAL, nominal)
    print(f"Natural lengths saved to {NOMINAL}")


# -- captions -------------------------------------------------------------------
def load_captions(video: str) -> dict[str, list[str]]:
    """Parse ``videos/captions/<video>.md``: ``## Scene`` headings, ``- card`` bullets."""
    path = CAPTIONS / f"{video}.md"
    cards: dict[str, list[str]] = {}
    scene = None
    if not path.exists():
        return cards
    for line in path.read_text(encoding="utf-8").splitlines():
        heading = re.match(r"^##\s+(S\d\d\w*)\s*$", line)
        if heading:
            scene = heading.group(1)
            cards[scene] = []
        elif scene and line.startswith("- "):
            cards[scene].append(line[2:].strip())
    return cards


def card_seconds(text: str) -> float:
    """Reading time of one caption card.

    Example:
        >>> card_seconds("short")
        2.2
        >>> round(card_seconds("x" * 140), 2)
        10.6
    """
    return max(MIN_CARD_S, len(text) / READING_CPS + 0.6)


def caption_schedule(cards: list[str], length_s: float) -> list[tuple[str, float, float]]:
    """Spread cards over a scene in proportion to their reading time.

    Returns ``(text, start_s, end_s)`` for each card.
    """
    if not cards:
        return []
    weights = [card_seconds(c) for c in cards]
    available = max(length_s - 2 * EDGE_S, 0.5)
    scale = available / sum(weights)
    schedule, start = [], EDGE_S
    for text, weight in zip(cards, weights):
        end = start + weight * scale
        schedule.append((text, start, end))
        start = end
    return schedule


def caption_image(text: str, width: int, height: int, path: Path) -> None:
    """Render one caption card as a band image (background colour, top rule, centred text)."""
    from PIL import Image, ImageDraw, ImageFont

    image = Image.new("RGB", (width, height), BACKGROUND)
    draw = ImageDraw.Draw(image)
    draw.rectangle([0, 0, width, max(1, height // 72)], fill=RULE_COLOR)
    margin = int(width * 0.06)
    for size_fraction in (0.27, 0.235, 0.20):            # shrink if more than 2 lines
        font = ImageFont.truetype(FONT_FILE, max(10, int(height * size_fraction)))
        average = font.getlength("abcdefghijklmnopqrstuvwxyz") / 26
        lines = textwrap.wrap(text, width=max(10, int((width - 2 * margin) / average)))
        if len(lines) <= 2 or size_fraction == 0.20:
            break
    fallback = ImageFont.truetype(FALLBACK_FONT_FILE, font.size)
    line_height = int(font.size * 1.3)
    top = (height - line_height * len(lines)) // 2 + height // 72
    for i, line in enumerate(lines):
        runs = font_runs(line, font, fallback)
        x = (width - sum(f.getlength(chunk) for chunk, f in runs)) / 2
        for chunk, run_font in runs:
            draw.text((x, top + i * line_height), chunk, font=run_font, fill=CAPTION_COLOR)
            x += run_font.getlength(chunk)
    image.save(path)


def font_runs(text: str, primary, fallback) -> list[tuple[str, object]]:
    """Split ``text`` into runs drawn with ``primary``, or ``fallback`` for glyphs it lacks.

    Segoe UI has no ⊗ (and similar maths symbols); without this they render as
    empty boxes. Segoe UI Symbol covers them.
    """
    covered = _font_cmap(primary.path)
    runs: list[tuple[str, object]] = []
    for character in text:
        chosen = primary if ord(character) in covered or character.isspace() else fallback
        if runs and runs[-1][1] is chosen:
            runs[-1] = (runs[-1][0] + character, chosen)
        else:
            runs.append((character, chosen))
    return runs


_CMAP_CACHE: dict[str, set[int]] = {}


def _font_cmap(font_path: str) -> set[int]:
    """Code points a font file can draw (cached)."""
    if font_path not in _CMAP_CACHE:
        from fontTools.ttLib import TTFont

        _CMAP_CACHE[font_path] = set(TTFont(font_path, fontNumber=0).getBestCmap())
    return _CMAP_CACHE[font_path]


# -- timing ---------------------------------------------------------------------
def find_audio(video: str, scene: str) -> Path | None:
    for extension in AUDIO_EXTENSIONS:
        candidate = AUDIO / video / f"{scene}{extension}"
        if candidate.exists():
            return candidate
    return None


def split_scales(target_s: float, play_s: float, wait_s: float) -> tuple[float, float]:
    """Split a scene's required stretch between animation and pause time.

    Animations get the square root of the overall stretch (bounded by
    ``PLAY_SCALE_RANGE``); pauses absorb the rest.

    Example:
        >>> [round(x, 2) for x in split_scales(40.0, 10.0, 10.0)]  # 2× overall
        [1.35, 2.65]
    """
    natural = play_s + wait_s
    play_scale = min(max((target_s / natural) ** 0.5, PLAY_SCALE_RANGE[0]), PLAY_SCALE_RANGE[1])
    if wait_s <= 1e-9:
        return target_s / play_s, 1.0
    wait_scale = (target_s - play_s * play_scale) / wait_s
    if wait_scale < MIN_WAIT_SCALE:            # very short clip: speed animations up instead
        wait_scale = MIN_WAIT_SCALE
        play_scale = (target_s - wait_s * wait_scale) / play_s
    return play_scale, wait_scale


def sync(video: str, hq: bool = True) -> None:
    """Fit each scene to its narration clip or caption reading time, then render."""
    stats = load_json(STATS)
    timings = load_json(TIMINGS)
    captions = load_captions(video)
    for scene in scene_names(video):
        key = f"{video}.{scene}"
        if key not in stats:
            raise SystemExit(f"No timing statistics for {key}; run 'measure' first.")
        natural = stats[key]["play"] + stats[key]["wait"]
        clip = find_audio(video, scene)
        if clip is not None:
            target, source = duration(clip) + TAIL_S, "audio"
        else:
            reading = sum(card_seconds(c) for c in captions.get(scene, [])) + 2 * EDGE_S
            target, source = max(natural, reading), "captions"
        play_scale, wait_scale = split_scales(target, stats[key]["play"], stats[key]["wait"])
        timings[key] = {"play": round(play_scale, 4), "wait": round(wait_scale, 4)}
        print(f"  {scene}: {natural:5.1f} s -> {target:5.1f} s ({source}); "
              f"animations x{play_scale:.2f}, pauses x{wait_scale:.2f}")
    save_json(TIMINGS, timings)
    render(video, hq=hq)


# -- assembly -------------------------------------------------------------------
def assemble(video: str, hq: bool = True, captions_on: bool = True) -> Path:
    """Join scene clips into ``videos/output/<video>.mp4`` with the caption band and audio."""
    segments_dir = BUILD / "segments" / video
    cards_dir = segments_dir / "captions"
    cards_dir.mkdir(parents=True, exist_ok=True)
    captions = load_captions(video) if captions_on else {}
    width, height = (1920, 1080) if hq else (854, 480)
    scene_h = int(round(height * SCENE_FRACTION / 2)) * 2
    scene_w = int(round(scene_h * 16 / 9 / 2)) * 2
    band_h = height - scene_h
    x_offset = (width - scene_w) // 2
    fps = "30" if hq else "15"
    segments = []
    for scene in scene_names(video):
        picture = scene_file(video, scene, hq)
        if not picture.exists():
            raise SystemExit(f"Missing render {picture}; run 'sync' or 'render' first.")
        length = duration(picture)
        clip = find_audio(video, scene)
        extra = max(0.0, duration(clip) + TAIL_S - length) if clip else 0.0
        total = length + extra

        inputs = ["-i", str(picture)]
        if clip is None:
            inputs += ["-f", "lavfi", "-t", f"{total:.3f}", "-i", "anullsrc=r=48000:cl=stereo"]
        else:
            inputs += ["-i", str(clip)]
        chain = [f"[0:v]scale={scene_w}:{scene_h}"]
        if extra > 0:                       # never cut speech short: hold the last frame
            chain.append(f"tpad=stop_mode=clone:stop_duration={extra:.3f}")
        chain.append(f"pad={width}:{height}:{x_offset}:0:color=0x{BACKGROUND[1:]}")
        graph = [",".join(chain) + "[b0]"]
        label = "b0"
        for index, (text, start, end) in enumerate(caption_schedule(captions.get(scene, []), total)):
            card = cards_dir / f"{scene}_{index:02d}_{'hq' if hq else 'lq'}.png"
            caption_image(text, width, band_h, card)
            inputs += ["-loop", "1", "-t", f"{total:.3f}", "-i", str(card)]
            stream = 2 + index
            graph.append(f"[{label}][{stream}:v]overlay=0:{scene_h}:enable='between(t,{start:.3f},{end:.3f})'[b{index + 1}]")
            label = f"b{index + 1}"
        segment = segments_dir / f"{scene}{'' if hq else '_lq'}.mp4"
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", *inputs, "-filter_complex", ";".join(graph),
                        "-map", f"[{label}]", "-map", "1:a:0", "-af", "apad", "-t", f"{total:.3f}",
                        "-c:v", "libx264", "-preset", "slow", "-crf", "18", "-pix_fmt", "yuv420p", "-r", fps,
                        "-c:a", "aac", "-b:a", "192k", "-ar", "48000", "-ac", "2", str(segment)], check=True)
        segments.append(segment)
    listing = segments_dir / f"list{'' if hq else '_lq'}.txt"
    listing.write_text("".join(f"file '{s.as_posix()}'\n" for s in segments), encoding="utf-8")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    final = OUTPUT / f"{video}{'' if hq else '_preview'}.mp4"
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-f", "concat", "-safe", "0", "-i", str(listing),
                    "-c", "copy", "-movflags", "+faststart", str(final)], check=True)
    print(f"Wrote {final} ({duration(final) / 60:.1f} min)")
    return final


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=["render", "measure", "sync", "assemble", "all"])
    parser.add_argument("video", help="module name, e.g. v1_rotations")
    parser.add_argument("scenes", nargs="*", help="optional scene names (render only)")
    parser.add_argument("--preview", action="store_true", help="480p15 instead of 1080p30")
    parser.add_argument("--hq", action="store_true", help="render: 1080p30 instead of 480p15")
    parser.add_argument("--no-captions", action="store_true", help="assemble without the caption band")
    args = parser.parse_args()
    hq = not args.preview
    if args.command == "render":
        render(args.video, args.scenes or None, hq=args.hq)
    elif args.command == "measure":
        measure(args.video)
    elif args.command == "sync":
        sync(args.video, hq=hq)
    elif args.command == "assemble":
        assemble(args.video, hq=hq, captions_on=not args.no_captions)
    else:
        if not all(f"{args.video}.{s}" in load_json(STATS) for s in scene_names(args.video)):
            measure(args.video)
        sync(args.video, hq=hq)
        assemble(args.video, hq=hq, captions_on=not args.no_captions)


if __name__ == "__main__":
    main()
