"""Render PowerShell-style terminal recordings of real command runs.

Each scene *really runs* its command, captures stdout/stderr lines with their
timestamps, and re-renders them as a Windows Terminal (PowerShell) window. The
output is encoded to MP4, and optionally a GIF, with ffmpeg. Long waits are
time-compressed, and the video notes when that happens.

Nothing shown is mocked: the text is the command's actual output on this
machine.
"""

from __future__ import annotations

import os
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

WIDTH, HEIGHT, FPS = 1280, 720, 30
BACKGROUND = (12, 12, 12)
TITLE_BAR = (32, 32, 32)
FOREGROUND = (204, 204, 204)
PROMPT_COLOR = (204, 204, 204)
COMMAND_COLOR = (249, 241, 165)
DIM = (118, 118, 118)
ERROR_COLOR = (231, 72, 86)
FONT_PATH = "C:/Windows/Fonts/consola.ttf"
FONT_SIZE = 19
LINE_HEIGHT = 25
MARGIN_X, TOP = 18, 48
TYPE_SPEED_CPS = 22.0
MAX_OUTPUT_GAP_S = 1.2     # longer silent gaps are compressed to this
PROMPT = "PS C:\\Users\\you\\Drone_Simulation> "


@dataclass
class Line:
    """One rendered terminal line."""

    text: str
    color: tuple[int, int, int] = FOREGROUND
    prompt_length: int = 0


@dataclass
class Step:
    """A command to type and run.

    Attributes:
        display: Text typed at the prompt.
        argv: Real command executed (None: only typed, e.g. ``Activate.ps1``).
        env: Extra environment variables for the run.
        keep: Optional filter; return False to hide an output line.
        pause_after_s: Idle time after the command finishes.
    """

    display: str
    argv: list[str] | None = None
    env: dict[str, str] = field(default_factory=dict)
    keep: object = None
    pause_after_s: float = 1.0
    cwd: str | None = None


REPO_ROOT = Path(__file__).resolve().parents[2]
PUBLIC_REPO_PATH = r"C:\Users\you\Drone_Simulation"


def mask_paths(line: str) -> str:
    """Replace this machine's paths with neutral ones, so recordings stay private."""
    home = Path.home()
    replacements = [
        (str(REPO_ROOT), PUBLIC_REPO_PATH),
        (REPO_ROOT.as_posix(), PUBLIC_REPO_PATH.replace("\\", "/")),
        (str(home), r"C:\Users\you"),
        (home.as_posix(), "C:/Users/you"),
    ]
    for real, public in replacements:
        line = line.replace(real, public)
    return line


def run_and_capture(step: Step) -> list[tuple[float, str, bool]]:
    """Run ``step.argv`` and return ``(seconds_since_start, line, is_stderr)``."""
    if step.argv is None:
        return []
    env = {**os.environ, "PYTHONUNBUFFERED": "1", "PYTHONIOENCODING": "utf-8", **step.env}
    start = time.perf_counter()
    process = subprocess.Popen(step.argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               env=env, cwd=step.cwd, text=True, encoding="utf-8",
                               errors="replace", bufsize=1)
    lines = []
    assert process.stdout is not None
    for raw in process.stdout:
        line = mask_paths(raw.rstrip("\n").replace("\t", "    "))
        if step.keep is None or step.keep(line):
            lines.append((time.perf_counter() - start, line, False))
    process.wait()
    end = time.perf_counter() - start
    lines.append((end, None, process.returncode != 0))  # sentinel: completion time
    return lines


class TerminalVideo:
    """Accumulates frames of a terminal session and pipes them to ffmpeg."""

    def __init__(self, output: Path, title: str = "Windows PowerShell") -> None:
        self.output = output
        self.title = title
        self.font = ImageFont.truetype(FONT_PATH, FONT_SIZE)
        self.title_font = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", 15)
        self.lines: list[Line] = []
        self.rows = (HEIGHT - TOP - 12) // LINE_HEIGHT
        self.char_width = self.font.getlength("M")
        self.columns = int((WIDTH - 2 * MARGIN_X) // self.char_width)
        output.parent.mkdir(parents=True, exist_ok=True)
        self.ffmpeg = subprocess.Popen(
            ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", f"{WIDTH}x{HEIGHT}", "-r", str(FPS), "-i", "-",
             "-c:v", "libx264", "-preset", "slow", "-crf", "20", "-pix_fmt", "yuv420p",
             "-movflags", "+faststart", str(output)],
            stdin=subprocess.PIPE)
        self.frame_count = 0
        self.overlay: Image.Image | None = None
        self.prompt_text = PROMPT

    # -- drawing -------------------------------------------------------------
    def _wrap(self, line: Line) -> list[Line]:
        text = line.text
        if len(text) <= self.columns:
            return [line]
        parts = [text[i:i + self.columns] for i in range(0, len(text), self.columns)]
        return [Line(parts[0], line.color, line.prompt_length)] + [Line(p, line.color) for p in parts[1:]]

    def render(self, cursor: bool = True) -> Image.Image:
        image = Image.new("RGB", (WIDTH, HEIGHT), BACKGROUND)
        draw = ImageDraw.Draw(image)
        draw.rectangle([0, 0, WIDTH, 34], fill=TITLE_BAR)
        draw.rounded_rectangle([10, 6, 230, 34], radius=6, fill=BACKGROUND)
        draw.text((22, 11), self.title, font=self.title_font, fill=FOREGROUND)
        # Window controls drawn as shapes (Consolas has no glyphs for them).
        draw.line([WIDTH - 128, 18, WIDTH - 116, 18], fill=DIM, width=1)
        draw.rectangle([WIDTH - 85, 12, WIDTH - 75, 22], outline=DIM, width=1)
        draw.line([WIDTH - 42, 12, WIDTH - 32, 22], fill=DIM, width=1)
        draw.line([WIDTH - 42, 22, WIDTH - 32, 12], fill=DIM, width=1)
        wrapped = [w for line in self.lines for w in self._wrap(line)]
        visible = wrapped[-self.rows:]
        y = TOP
        for line in visible:
            x = MARGIN_X
            if line.prompt_length:
                prompt = line.text[:line.prompt_length]
                draw.text((x, y), prompt, font=self.font, fill=PROMPT_COLOR)
                x += self.font.getlength(prompt)
                draw.text((x, y), line.text[line.prompt_length:], font=self.font, fill=COMMAND_COLOR)
            else:
                draw.text((x, y), line.text, font=self.font, fill=line.color)
            y += LINE_HEIGHT
        if cursor and visible and (self.frame_count // (FPS // 2)) % 2 == 0:
            last = visible[-1]
            cx = MARGIN_X + self.font.getlength(last.text)
            cy = TOP + (len(visible) - 1) * LINE_HEIGHT
            draw.rectangle([cx + 1, cy + 3, cx + self.char_width - 1, cy + LINE_HEIGHT - 3],
                           fill=FOREGROUND)
        if self.overlay is not None:
            image.paste(self.overlay, (0, 0), self.overlay)
        return image

    def emit(self, seconds: float, cursor: bool = True) -> None:
        for _ in range(max(1, round(seconds * FPS))):
            frame = self.render(cursor)
            assert self.ffmpeg.stdin is not None
            self.ffmpeg.stdin.write(frame.tobytes())
            self.frame_count += 1

    # -- scripting -----------------------------------------------------------
    def prompt(self) -> None:
        self.lines.append(Line(self.prompt_text, prompt_length=len(self.prompt_text)))

    def type_command(self, text: str) -> None:
        for char in text:
            self.lines[-1].text += char
            self.emit(1.0 / TYPE_SPEED_CPS)
        self.emit(0.35)

    def play_step(self, step: Step) -> None:
        self.type_command(step.display)
        captured = run_and_capture(step)
        previous = 0.0
        for timestamp, text, failed in captured:
            gap = min(timestamp - previous, MAX_OUTPUT_GAP_S)
            self.emit(max(gap, 1.0 / FPS), cursor=False)
            previous = timestamp
            if text is None:
                if failed:
                    self.lines.append(Line("(command failed)", ERROR_COLOR))
                break
            self.lines.append(Line(text, ERROR_COLOR if "Error" in text else FOREGROUND))
        self.prompt()
        self.emit(step.pause_after_s)

    def note(self, text: str) -> None:
        """Print a dim annotation line (e.g. that output was abridged)."""
        self.lines.append(Line(text, DIM))

    def show_overlay(self, overlay: Image.Image, seconds: float) -> None:
        self.overlay = overlay
        self.emit(seconds, cursor=False)
        self.overlay = None

    def close(self) -> None:
        assert self.ffmpeg.stdin is not None
        self.ffmpeg.stdin.close()
        self.ffmpeg.wait()


def figure_windows_overlay(images: list[Path], count: int) -> Image.Image:
    """Tile the first ``count`` figure images as desktop windows (3×2 grid)."""
    overlay = Image.new("RGBA", (WIDTH, HEIGHT), (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    title_font = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", 13)
    columns, rows = 3, 2
    cell_w, cell_h = WIDTH // columns, HEIGHT // rows
    for index, path in enumerate(images[:count]):
        x0, y0 = (index % columns) * cell_w, (index // columns) * cell_h
        draw.rectangle([x0, y0, x0 + cell_w - 2, y0 + cell_h - 2], fill=(255, 255, 255, 255),
                       outline=(90, 90, 90, 255))
        draw.rectangle([x0, y0, x0 + cell_w - 2, y0 + 24], fill=(240, 240, 240, 255))
        draw.text((x0 + 8, y0 + 4), f"Figure {index + 1}", font=title_font, fill=(20, 20, 20, 255))
        figure = Image.open(path).convert("RGBA")
        figure.thumbnail((cell_w - 12, cell_h - 34))
        overlay.paste(figure, (x0 + (cell_w - figure.width) // 2, y0 + 28 + (cell_h - 34 - figure.height) // 2))
    return overlay


def to_gif(mp4: Path, gif: Path, width: int = 820, fps: int = 10) -> None:
    """Convert an MP4 to an optimized GIF with a generated palette."""
    filters = f"fps={fps},scale={width}:-1:flags=lanczos"
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", str(mp4), "-vf",
                    f"{filters},split[a][b];[a]palettegen=max_colors=64[p];[b][p]paletteuse=dither=none",
                    str(gif)], check=True)
