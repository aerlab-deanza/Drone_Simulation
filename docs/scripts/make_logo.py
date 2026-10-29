"""Build the transparent logo variants and favicon from the AERL logo.

The source logo is white/grey line art on a solid deep-teal background. This
script removes the background (recovering a proper alpha channel) and writes:

* ``logo-dark.png`` — original colours, for the dark theme;
* ``logo-light.png`` — lines recoloured to the lab's deep teal, for the light theme;
* ``favicon.png`` — the gear-and-magnifier mark on a teal rounded square.

Run from the repository root::

    python docs/scripts/make_logo.py docs/_static/aerl_logo_source.png
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

OUT = Path(__file__).resolve().parents[1] / "_static"
BACKGROUND = np.array([0, 52, 61], dtype=float)        # #00343d
TEAL = np.array([0, 52, 61], dtype=float)              # light-theme line colour
TEAL_ACCENT = np.array([11, 127, 140], dtype=float)    # light-theme stand-in for cyan
WHITE = np.array([255, 255, 255], dtype=float)


def unmix(rgb: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Split a flat-background image into foreground colour and alpha.

    Each pixel is modelled as ``alpha * colour + (1 - alpha) * background``.
    Alpha is the largest fraction any channel has moved away from the
    background, which is exact for colours on the line between the two.
    """
    diff = rgb - BACKGROUND
    headroom = np.where(diff >= 0, WHITE - BACKGROUND, BACKGROUND)   # distance to 255 or to 0
    alpha = np.clip(np.max(np.abs(diff) / np.maximum(headroom, 1e-6), axis=-1), 0, 1)
    alpha = np.where(alpha < 0.06, 0.0, alpha)                        # drop compression noise
    colour = BACKGROUND + diff / np.maximum(alpha[..., None], 1e-6)
    return np.clip(colour, 0, 255), alpha


def to_image(colour: np.ndarray, alpha: np.ndarray) -> Image.Image:
    rgba = np.dstack([colour, alpha * 255]).astype(np.uint8)
    return Image.fromarray(rgba, "RGBA")


def crop_to_content(image: Image.Image, margin: int = 12) -> Image.Image:
    box = image.getchannel("A").point(lambda a: 255 if a > 20 else 0).getbbox()
    left, top, right, bottom = box
    return image.crop((max(0, left - margin), max(0, top - margin),
                       min(image.width, right + margin), min(image.height, bottom + margin)))


def main(source: Path) -> None:
    original = Image.open(source).convert("RGB")
    upscaled = original.resize((original.width * 2, original.height * 2), Image.LANCZOS)
    rgb = np.asarray(upscaled, dtype=float)
    colour, alpha = unmix(rgb)

    dark = crop_to_content(to_image(colour, alpha))
    dark.save(OUT / "logo-dark.png")

    # Light theme: neutral (white/grey) strokes -> deep teal; cyan -> darker teal; red stays.
    saturation = colour.max(axis=-1) - colour.min(axis=-1)
    is_red = (colour[..., 0] > 180) & (colour[..., 1] < 140)
    is_cyan = (saturation > 60) & ~is_red
    # The magnifier glass is a pale cyan fill (#c9f6fd): keep it as a light tint.
    is_glass = ((colour[..., 2] - colour[..., 0]) > 25) & (colour.mean(axis=-1) > 170) & ~is_cyan
    light_colour = np.where(is_red[..., None], colour,
                            np.where(is_cyan[..., None], TEAL_ACCENT,
                                     np.where(is_glass[..., None], colour * 0.9, TEAL)))
    # The grey "AERL" letters unmix to ~85 % alpha; draw strokes fully opaque on white.
    light_alpha = np.where(is_glass, alpha, np.clip(alpha / 0.85, 0, 1))
    light = crop_to_content(to_image(light_colour, light_alpha))
    light.save(OUT / "logo-light.png")

    # Favicon: the gear + magnifier (left part of the logo) on a teal rounded square.
    mark = crop_to_content(to_image(colour, alpha).crop((0, 0, int(upscaled.width * 0.47), upscaled.height)))
    size = 256
    icon = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    ImageDraw.Draw(icon).rounded_rectangle([0, 0, size - 1, size - 1], radius=48, fill=(0, 52, 61, 255))
    mark.thumbnail((int(size * 0.84), int(size * 0.84)), Image.LANCZOS)
    icon.alpha_composite(mark, ((size - mark.width) // 2, (size - mark.height) // 2))
    icon.resize((64, 64), Image.LANCZOS).save(OUT / "favicon.png")
    print(f"wrote logo-dark.png {dark.size}, logo-light.png {light.size}, favicon.png")


if __name__ == "__main__":
    main(Path(sys.argv[1]))
