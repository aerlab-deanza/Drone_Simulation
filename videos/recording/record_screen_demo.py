"""Record a real screen capture of the offline demo, with window interaction.

Run from the repository root, then don't touch the mouse or keyboard
(about 90 s)::

    python videos/recording/record_screen_demo.py

The script really runs the demo and controls the mouse to show what a user can
do: it opens a fresh PowerShell window, types ``python -m drone_sim``, waits
for the six plot windows, then:

1. maximizes the 3D figure (double-click on its title bar) and rotates the 3D
   view with mouse drags, then restores it;
2. resizes another figure window by dragging its corner, then maximizes and
   restores it;
3. closes the plot windows, so the prompt comes back.

The desktop is captured with ffmpeg (gdigrab). The taskbar is cropped away
and the result is written to ``docs/_static/videos/static_command.mp4`` (plus
the README GIF). The terminal prompt is masked to
``C:\\Users\\you\\Drone_Simulation``.

Safety: pyautogui's fail-safe is on. Slam the mouse into the top-left corner
of the screen to abort immediately.
"""

from __future__ import annotations

import ctypes
import os
import subprocess
import sys
import time
from pathlib import Path

ctypes.windll.shcore.SetProcessDpiAwareness(2)   # physical-pixel coordinates

import pyautogui  # noqa: E402
import pygetwindow as gw  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from terminal_recorder import to_gif  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
OUT = REPO / "docs" / "_static" / "videos"
RAW = REPO / "videos" / "build" / "screen_raw.mp4"
TERMINAL_TITLE = "Windows PowerShell - drone_sim demo"
SCALE = ctypes.windll.user32.GetDpiForSystem() / 96.0
TASKBAR_PX = int(round(48 * SCALE))

pyautogui.FAILSAFE = True
pyautogui.PAUSE = 0.05


def wait_for_window(title: str, timeout_s: float = 90.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        found = [w for w in gw.getWindowsWithTitle(title) if w.title == title]
        if found:
            return found[0]
        time.sleep(0.3)
    raise TimeoutError(f"window {title!r} did not appear")


def title_bar(window) -> tuple[int, int]:
    """A point on the window's title bar, clear of its buttons."""
    return window.left + window.width // 3, window.top + int(16 * SCALE)


def glide(x: float, y: float, seconds: float = 0.6) -> None:
    pyautogui.moveTo(int(x), int(y), duration=seconds, tween=pyautogui.easeInOutQuad)


def drag(path: list[tuple[float, float]], seconds_per_leg: float = 1.4) -> None:
    """Hold the left button and move along ``path`` (rotates a 3D Matplotlib view)."""
    glide(*path[0], 0.5)
    pyautogui.mouseDown()
    for x, y in path[1:]:
        pyautogui.moveTo(int(x), int(y), duration=seconds_per_leg, tween=pyautogui.easeInOutSine)
    pyautogui.mouseUp()


def toggle_maximize(window) -> None:
    glide(*title_bar(window), 0.7)
    time.sleep(0.25)
    pyautogui.doubleClick()
    time.sleep(1.2)


def start_capture() -> subprocess.Popen:
    RAW.parent.mkdir(parents=True, exist_ok=True)
    return subprocess.Popen(
        ["ffmpeg", "-loglevel", "error", "-y", "-f", "gdigrab", "-framerate", "30", "-draw_mouse", "1",
         "-i", "desktop", "-c:v", "libx264", "-preset", "ultrafast", "-crf", "16", "-pix_fmt", "yuv420p",
         str(RAW)], stdin=subprocess.PIPE)


def stop_capture(process: subprocess.Popen) -> None:
    if process.poll() is None:
        process.communicate(b"q", timeout=30)


def open_terminal() -> None:
    setup = "; ".join([
        f"Set-Location '{REPO}'",
        "& '.venv\\Scripts\\Activate.ps1'",
        "Remove-Item Env:MPLBACKEND -ErrorAction SilentlyContinue",
        f"$Host.UI.RawUI.WindowTitle = '{TERMINAL_TITLE}'",
        "function global:prompt { '(.venv) PS C:\\Users\\you\\Drone_Simulation> ' }",
        "Clear-Host",
    ])
    env = {k: v for k, v in os.environ.items() if k != "MPLBACKEND"}
    subprocess.Popen(["powershell.exe", "-NoExit", "-NoLogo", "-Command", setup],
                     creationflags=subprocess.CREATE_NEW_CONSOLE, env=env)


def post_process() -> None:
    """Crop the taskbar, fit into 1280×720, and write the MP4 and README GIF."""
    final = OUT / "static_command.mp4"
    width, height = pyautogui.size()
    crop_h = height - TASKBAR_PX
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", str(RAW), "-vf",
                    f"crop={width}:{crop_h}:0:0,scale=-2:720,pad=1280:720:(ow-iw)/2:0:color=0x0c0c0c,fps=30",
                    "-c:v", "libx264", "-preset", "slow", "-crf", "24", "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart", "-an", str(final)], check=True)
    to_gif(final, OUT / "static_command.gif", width=640, fps=5)
    print(f"Wrote {final} and static_command.gif")


def main() -> None:
    capture = None
    try:
        open_terminal()
        terminal = wait_for_window(TERMINAL_TITLE, 30)
        terminal.activate()
        terminal.maximize()
        time.sleep(1.0)
        pyautogui.keyDown("ctrl")                    # larger console font, readable in the video
        pyautogui.scroll(4)
        pyautogui.keyUp("ctrl")
        time.sleep(0.8)

        capture = start_capture()
        time.sleep(1.5)
        pyautogui.write("python -m drone_sim", interval=0.07)
        time.sleep(0.4)
        pyautogui.press("enter")

        figure_3d = wait_for_window("Figure 1", 120)
        for n in range(2, 7):
            wait_for_window(f"Figure {n}", 30)
        time.sleep(2.5)                              # let the viewer see all six windows

        # 1. Maximize the 3D figure and rotate the view with the mouse.
        figure_3d.activate()
        toggle_maximize(figure_3d)
        width, height = pyautogui.size()
        cx, cy = width * 0.47, height * 0.52
        drag([(cx - 260, cy), (cx + 300, cy), (cx + 300, cy - 180), (cx - 120, cy + 120)])
        time.sleep(0.6)
        drag([(cx + 200, cy + 40), (cx - 320, cy - 60)], seconds_per_leg=1.8)
        time.sleep(1.2)
        toggle_maximize(gw.getWindowsWithTitle("Figure 1")[0])

        # 2. Resize another window by its corner, then maximize and restore it.
        figure = gw.getWindowsWithTitle("Figure 2")[0]
        figure.activate()
        time.sleep(0.5)
        corner = (figure.right - int(4 * SCALE), figure.bottom - int(4 * SCALE))
        glide(*corner, 0.8)
        pyautogui.mouseDown()
        pyautogui.moveTo(corner[0] + int(260 * SCALE), corner[1] + int(170 * SCALE), duration=1.6,
                         tween=pyautogui.easeInOutQuad)
        time.sleep(0.6)
        pyautogui.moveTo(*corner, duration=1.2, tween=pyautogui.easeInOutQuad)
        pyautogui.mouseUp()
        time.sleep(0.6)
        figure = gw.getWindowsWithTitle("Figure 2")[0]
        toggle_maximize(figure)
        time.sleep(1.0)
        toggle_maximize(gw.getWindowsWithTitle("Figure 2")[0])

        # 3. Close the plot windows; the demo exits and the prompt returns.
        for n in range(6, 0, -1):
            for window in gw.getWindowsWithTitle(f"Figure {n}"):
                if window.title == f"Figure {n}":
                    window.close()
                    time.sleep(0.35)
        time.sleep(2.5)
    finally:
        if capture is not None:
            stop_capture(capture)
        for window in gw.getWindowsWithTitle(TERMINAL_TITLE):
            window.close()
    post_process()


if __name__ == "__main__":
    main()
