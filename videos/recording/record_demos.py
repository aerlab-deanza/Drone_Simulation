"""Record the command-line demo videos for the documentation.

Run from the repository root (needs ffmpeg on PATH)::

    python videos/recording/record_demos.py            # all scenes
    python videos/recording/record_demos.py lqr        # one scene

Outputs go to ``docs/_static/videos``: ``install.mp4`` and ``lqr_command.mp4``.
The static demo is a real screen capture instead: see ``record_screen_demo.py``.

Every command is really executed, in a scratch folder so the repository's
``outputs/`` stays untouched. The install scene creates a fresh virtual
environment and installs the package into it.
"""

from __future__ import annotations

import shutil
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from terminal_recorder import Step, TerminalVideo, figure_windows_overlay  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
OUT = REPO / "docs" / "_static" / "videos"
PYTHON = sys.executable
AGG = {"MPLBACKEND": "Agg"}


def not_matplotlib_noise(line: str) -> bool:
    """Hide warnings caused only by the headless (Agg) backend used for recording."""
    return "non-interactive" not in line and "plt.show()" not in line and line.strip() != ""


def scene_install(work: Path) -> None:
    video = TerminalVideo(OUT / "install.mp4")
    venv = work / ".venv"
    venv_python = venv / "Scripts" / "python.exe"
    video.prompt()
    video.emit(0.8)
    video.play_step(Step("python -m venv .venv", [PYTHON, "-m", "venv", str(venv)], pause_after_s=0.6))
    video.type_command(".venv\\Scripts\\Activate.ps1")
    video.prompt_text = "(.venv) " + video.prompt_text
    video.prompt()
    video.emit(0.6)

    def keep_pip(line: str) -> bool:
        return line.startswith(("Obtaining", "Successfully installed", "Installing collected"))

    video.play_step(Step('python -m pip install -e ".[test]"',
                         [str(venv_python), "-m", "pip", "install", "--disable-pip-version-check",
                          "-e", f"{REPO}[test]"], keep=keep_pip, pause_after_s=0.2))
    video.lines.insert(-1, video.lines[-1].__class__("(download progress omitted)", (118, 118, 118)))
    video.emit(0.8)
    video.play_step(Step("python -m pytest -q", [str(venv_python), "-m", "pytest", "-q", "-p",
                                                 "no:warnings"], env=AGG,
                         cwd=str(REPO), keep=lambda l: l.strip() != "", pause_after_s=2.5))
    video.close()


def scene_lqr(work: Path) -> None:
    video = TerminalVideo(OUT / "lqr_command.mp4")
    (work / "outputs").mkdir(exist_ok=True)
    video.prompt()
    video.emit(0.8)
    video.play_step(Step("python -m drone_sim.control.lqr", [PYTHON, "-m", "drone_sim.control.lqr"],
                         env=AGG, cwd=str(work), keep=not_matplotlib_noise, pause_after_s=0.5))
    video.show_overlay(figure_windows_overlay([work / "outputs" / "lqr_response.png"], 1), 5.0)
    video.close()


SCENES = {"install": scene_install, "lqr": scene_lqr}


def main() -> None:
    selected = sys.argv[1:] or list(SCENES)
    work = Path(tempfile.mkdtemp(prefix="drone_sim_recording_"))
    (work / "outputs").mkdir()
    try:
        for name in selected:
            print(f"Recording {name}...")
            SCENES[name](work)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    print(f"Done. Videos are in {OUT}")


if __name__ == "__main__":
    main()
