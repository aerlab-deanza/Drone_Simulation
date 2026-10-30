"""Record a real screen tour of the documentation and turn it into the README GIF.

Run from the repository root, with the docs built and served on port 8765::

    python -m http.server 8765 --directory docs/_build/html     # in another terminal
    python videos/recording/record_docs_tour.py

Then don't touch the mouse or keyboard (about 3 minutes). The script opens the
docs in a clean Microsoft Edge app window (fresh profile, no tabs or address
bar) and:

1. switches the colour theme from the toggle menu to light, then back to dark;
2. walks through **every page** in reading order with the → key, which the
   theme supports, scrolling a little on each;
3. stops longer on the static-demo recording, the live-demo video, and the
   theory video on the Euler-angles page, playing each for a while.

Element positions come from the live page through Edge's DevTools protocol,
so the mouse lands exactly on buttons and videos. The screen is captured
with ffmpeg (gdigrab). The title bar and taskbar are cropped, and the result is written to
``docs/_static/videos/docs_tour.gif`` (and an MP4 alongside it).

Safety: pyautogui's fail-safe is on. Slam the mouse into the top-left corner
to abort.
"""

from __future__ import annotations

import ctypes
import ctypes.wintypes
import json
import shutil
import subprocess
import tempfile
import time
import urllib.request
from pathlib import Path

ctypes.windll.shcore.SetProcessDpiAwareness(2)   # physical-pixel coordinates

import pyautogui  # noqa: E402
import pygetwindow as gw  # noqa: E402
import websocket  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
BUILD_HTML = REPO / "docs" / "_build" / "html"
OUT = REPO / "docs" / "_static" / "videos"
RAW = REPO / "videos" / "build" / "docs_tour_raw.mp4"
EDGE = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
BASE = "http://localhost:8765/"
PORT = 9333
SCALE = ctypes.windll.user32.GetDpiForSystem() / 96.0
TASKBAR_PX = int(round(48 * SCALE))
TITLEBAR_PX = int(round(23 * SCALE))      # Edge app-window title bar

pyautogui.FAILSAFE = True
pyautogui.PAUSE = 0.03


# -- Edge DevTools ----------------------------------------------------------------
class Page:
    """Minimal DevTools client for the single page of the app window."""

    def __init__(self) -> None:
        for _ in range(60):
            try:
                targets = json.load(urllib.request.urlopen(f"http://127.0.0.1:{PORT}/json"))
                page = next(t for t in targets
                            if t.get("type") == "page" and t.get("url", "").startswith(BASE))
                break
            except Exception:
                time.sleep(0.5)
        else:
            raise RuntimeError("Edge DevTools endpoint did not come up")
        self.ws = websocket.create_connection(page["webSocketDebuggerUrl"], timeout=20)
        self.next_id = 0

    def js(self, expression: str):
        self.next_id += 1
        self.ws.send(json.dumps({"id": self.next_id, "method": "Runtime.evaluate",
                                 "params": {"expression": expression, "returnByValue": True,
                                            "awaitPromise": True}}))
        while True:
            message = json.loads(self.ws.recv())
            if message.get("id") == self.next_id:
                return message.get("result", {}).get("result", {}).get("value")

    def path(self) -> str:
        return self.js("location.pathname")

    def wait_loaded(self) -> None:
        for _ in range(100):
            if self.js("document.readyState") == "complete":
                return
            time.sleep(0.1)


def viewport_origin(window) -> tuple[float, float]:
    """Screen position (physical px) of the page's top-left corner."""
    rect = ctypes.wintypes.RECT()
    ctypes.windll.user32.GetClientRect(window._hWnd, ctypes.byref(rect))
    point = ctypes.wintypes.POINT(0, 0)
    ctypes.windll.user32.ClientToScreen(window._hWnd, ctypes.byref(point))
    return point.x, point.y


def element_center(page: Page, origin, selector: str):
    """Screen coordinates of the centre of the first element matching ``selector``."""
    box = page.js(f"""(() => {{ const e = [...document.querySelectorAll({json.dumps(selector)})]
            .find(el => el.getBoundingClientRect().width > 0);
        if (!e) return null; const r = e.getBoundingClientRect();
        return [r.left + r.width / 2, r.top + r.height / 2, window.devicePixelRatio,
                window.outerHeight - window.innerHeight]; }})()""")
    if box is None:
        return None
    x, y, dpr, chrome = box
    return origin[0] + x * dpr, origin[1] + (chrome + y) * dpr


# -- motion helpers ------------------------------------------------------------------
def glide(x: float, y: float, seconds: float = 0.6) -> None:
    pyautogui.moveTo(int(x), int(y), duration=seconds, tween=pyautogui.easeInOutQuad)


def scroll_view(page: Page, selector: str, block: str = "center") -> None:
    page.js(f"document.querySelector({json.dumps(selector)})"
            f".scrollIntoView({{behavior: 'smooth', block: '{block}'}})")
    time.sleep(1.2)


def gentle_scroll(clicks: int = -4, pause: float = 0.8) -> None:
    for _ in range(abs(clicks)):
        pyautogui.scroll(-120 if clicks < 0 else 120)
        time.sleep(0.12)
    time.sleep(pause)


def play_video(page: Page, origin, selector: str, seconds: float) -> None:
    """Scroll a video into view, click it like a user, and let it play."""
    scroll_view(page, selector)
    target = element_center(page, origin, selector)
    if target is None:
        return
    glide(*target, 0.7)
    time.sleep(0.3)
    pyautogui.click()
    time.sleep(seconds)
    if "iframe" not in selector:
        page.js(f"document.querySelector({json.dumps(selector)}).pause()")
    else:                                   # YouTube: click again to pause
        pyautogui.click()
        time.sleep(0.4)
        # The player now holds keyboard focus; click the page heading to take it back.
        heading = element_center(page, origin, "article h1")
        if heading is not None:
            glide(*heading, 0.5)
            pyautogui.click()
    time.sleep(0.4)


# -- capture -------------------------------------------------------------------------
def start_capture() -> subprocess.Popen:
    RAW.parent.mkdir(parents=True, exist_ok=True)
    return subprocess.Popen(
        ["ffmpeg", "-loglevel", "error", "-y", "-f", "gdigrab", "-framerate", "24", "-draw_mouse", "1",
         "-i", "desktop", "-c:v", "libx264", "-preset", "ultrafast", "-crf", "18", "-pix_fmt", "yuv420p",
         str(RAW)], stdin=subprocess.PIPE)


def stop_capture(process: subprocess.Popen) -> None:
    if process.poll() is None:
        process.communicate(b"q", timeout=60)


def post_process() -> None:
    """Crop the title bar and taskbar; write an MP4 and a README-sized GIF (sped up 1.8×)."""
    width, height = pyautogui.size()
    crop = f"crop={width}:{height - TASKBAR_PX - TITLEBAR_PX}:0:{TITLEBAR_PX}"
    mp4 = OUT / "docs_tour.mp4"
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", str(RAW), "-vf",
                    f"{crop},scale=-2:720,fps=24", "-c:v", "libx264", "-preset", "slow", "-crf", "26",
                    "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-an", str(mp4)], check=True)
    gif = OUT / "docs_tour.gif"
    filters = f"{crop},setpts=PTS/1.8,fps=6,scale=720:-1:flags=lanczos"
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", str(RAW), "-vf",
                    f"{filters},split[a][b];[a]palettegen=max_colors=64:stats_mode=diff[p];"
                    "[b][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle", str(gif)], check=True)
    print(f"Wrote {mp4} and {gif} ({gif.stat().st_size / 1e6:.1f} MB)")


# -- the tour ------------------------------------------------------------------------
def main() -> None:
    profile = Path(tempfile.mkdtemp(prefix="docs_tour_edge_"))
    edge = subprocess.Popen([EDGE, f"--app={BASE}index.html", f"--remote-debugging-port={PORT}",
                             f"--user-data-dir={profile}", "--inprivate", "--disable-extensions", "--disable-sync",
                             "--no-first-run", "--no-default-browser-check",
                             "--disable-features=msEdgeSidebar,EdgeCollections"])
    capture = None
    try:
        page = Page()
        page.wait_loaded()
        page.js("localStorage.setItem('mode', 'dark'); document.documentElement.dataset.mode = 'dark';"
                "document.documentElement.dataset.theme = 'dark';")
        window = None
        for _ in range(40):
            matches = [w for w in gw.getAllWindows() if "Drone Simulation" in w.title]
            if matches:
                window = matches[0]
                break
            time.sleep(0.3)
        if window is None:
            raise RuntimeError("Edge app window not found")
        window.activate()
        window.maximize()
        time.sleep(1.5)
        origin = viewport_origin(window)

        capture = start_capture()
        time.sleep(2.0)

        # 1. Theme change: open the colour-mode menu, pick Light, then Dark.
        for wanted in ("light", "dark"):
            for _ in range(3):
                glide(*element_center(page, origin, "button.theme-switch-button"), 0.8)
                pyautogui.click()
                time.sleep(0.8)
                item = element_center(page, origin, f'button.theme-change-button[data-mode="{wanted}"]')
                if item is not None:
                    glide(*item, 0.5)
                    time.sleep(0.2)
                    pyautogui.click()
                time.sleep(0.9)
                if page.js("document.documentElement.dataset.theme") == wanted:
                    break
            time.sleep(1.8)
        glide(origin[0] + 700 * SCALE, origin[1] + 450 * SCALE, 0.6)
        gentle_scroll(-5, 1.5)

        # 2. Every page, in reading order, with the → key (the "next" link if a key press is lost).
        visited = [page.path()]
        while True:
            previous = page.path()
            next_href = page.js("(document.querySelector('a.right-next') || {}).href || null")
            if not next_href:
                break                                     # last page reached
            page.js("document.activeElement && document.activeElement.blur()")
            pyautogui.press("right")
            for _ in range(30):
                time.sleep(0.1)
                if page.path() != previous:
                    break
            else:
                page.js(f"location.href = {json.dumps(next_href)}")
                for _ in range(50):
                    time.sleep(0.1)
                    if page.path() != previous:
                        break
            page.wait_loaded()
            current = page.path()
            if current == previous or current in visited:
                break
            visited.append(current)
            print("page", len(visited), current, flush=True)
            time.sleep(0.5)
            glide(origin[0] + 900 * SCALE, origin[1] + 500 * SCALE, 0.3)

            if current.endswith("demos/static.html"):
                play_video(page, origin, 'video[src*="static_command"]', 7.0)
                play_video(page, origin, 'video[src*="static_replay"]', 7.0)
            elif current.endswith("demos/live.html"):
                play_video(page, origin, ".video-embed iframe", 9.0)
            elif current.endswith("theory/euler-angles.html"):
                play_video(page, origin, "video.demo-video", 7.0)
                scroll_view(page, "details.sd-dropdown")
                target = element_center(page, origin, "details.sd-dropdown summary")
                if target is not None:
                    glide(*target, 0.6)
                    pyautogui.click()                      # open a derivation
                    time.sleep(1.8)
            elif current.endswith("videos/index.html"):
                play_video(page, origin, 'video[src*="v3_fusion"]', 7.0)
            elif "/api/generated/" in current:
                gentle_scroll(-2, 0.3)                     # quick look at each module page
                continue
            else:
                gentle_scroll(-4, 0.9)
            time.sleep(0.6)
        time.sleep(1.5)
    finally:
        if capture is not None:
            stop_capture(capture)
        edge.terminate()
        time.sleep(1.0)
        shutil.rmtree(profile, ignore_errors=True)
    post_process()
    print(f"Visited {len(visited)} pages")


if __name__ == "__main__":
    main()
