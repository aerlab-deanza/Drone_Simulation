"""Tile frames from every rendered scene of a video into one image, for review.

Usage::

    python videos/contact_sheet.py v1_rotations [480p15]

Writes ``videos/build/contact_<video>_<n>.png`` (several pages if needed),
taking frames at 35 %, 70 % and 97 % of each scene.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

BUILD = Path(__file__).resolve().parent / "build"


def duration(path: Path) -> float:
    out = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of",
                          "csv=p=0", str(path)], capture_output=True, text=True, check=True)
    return float(out.stdout.strip())


def frame(path: Path, t: float) -> Image.Image:
    raw = subprocess.run(["ffmpeg", "-loglevel", "error", "-ss", f"{t:.2f}", "-i", str(path), "-frames:v", "1",
                          "-f", "image2pipe", "-vcodec", "png", "-"], capture_output=True, check=True).stdout
    from io import BytesIO
    return Image.open(BytesIO(raw)).convert("RGB")


def main() -> None:
    video = sys.argv[1]
    quality = sys.argv[2] if len(sys.argv) > 2 else "480p15"
    folder = BUILD / "media" / "videos" / video / quality
    scenes = sorted(p for p in folder.glob("S*.mp4"))
    font = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", 18)
    tiles = []
    for scene in scenes:
        length = duration(scene)
        for fraction in (0.35, 0.70, 0.97):
            image = frame(scene, length * fraction).resize((480, 270))
            ImageDraw.Draw(image).text((6, 2), f"{scene.stem} {fraction:.0%} ({length:.0f}s)", font=font,
                                       fill=(255, 255, 0))
            tiles.append(image)
    per_page = 15
    for page in range(0, len(tiles), per_page):
        chunk = tiles[page:page + per_page]
        sheet = Image.new("RGB", (480 * 3, 270 * ((len(chunk) + 2) // 3)), (40, 40, 40))
        for i, tile in enumerate(chunk):
            sheet.paste(tile, ((i % 3) * 480, (i // 3) * 270))
        out = BUILD / f"contact_{video}_{page // per_page + 1}.png"
        sheet.save(out)
        print(out)


if __name__ == "__main__":
    main()
