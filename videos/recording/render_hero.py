"""Render the documentation front-page banner: a looping 3D figure-eight flight.

Run from the repository root::

    python videos/recording/render_hero.py

Writes ``docs/_static/videos/hero_loop.mp4`` (silent, 960×540) and
``docs/_static/figures/hero_poster.png`` (shown before the video loads), in the
AERL palette: deep teal background, cyan flight path, red reference.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FFMpegWriter  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from drone_sim.simulation.pipeline import run_pipeline  # noqa: E402
from render_replay import frame_geometry  # noqa: E402

TEAL = "#00343d"
CYAN = "#5ce1e6"
RED = "#ff3131"
WHITE = "#e8f4f6"
FPS = 24
SPEED = 0.8                       # playback speed vs real time


def main() -> None:
    log = run_pipeline(controller_type="pid")
    t, pos, ref, att = log["t"], log["pos"], log["pos_des"], log["att_true"]
    stride = max(1, int(round(SPEED / (FPS * (t[1] - t[0])))))
    indices = np.arange(0, len(t), stride)

    fig = plt.figure(figsize=(9.6, 5.4), dpi=100, facecolor=TEAL)
    ax = fig.add_axes([-0.12, -0.18, 1.24, 1.36], projection="3d", facecolor=TEAL)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_pane_color((0, 0, 0, 0))
        axis._axinfo["grid"].update(color=(0.36, 0.88, 0.90, 0.10), linewidth=0.6)
        axis.line.set_color((0, 0, 0, 0))
        axis.set_ticklabels([])
        for tick in axis.get_major_ticks():
            tick.tick1line.set_visible(False)
            tick.tick2line.set_visible(False)
    ax.tick_params(axis="both", which="both", length=0, colors=(0, 0, 0, 0))
    margin = 0.4
    ax.set_xlim(pos[:, 0].min() - margin, pos[:, 0].max() + margin)
    ax.set_ylim(pos[:, 1].min() - margin, pos[:, 1].max() + margin)
    ax.set_zlim(1.6, 3.9)
    ax.set_box_aspect((1.4, 1.0, 0.75))

    ax.plot(*ref.T, "--", color=RED, lw=1.4, alpha=0.75)
    trail, = ax.plot([], [], [], color=CYAN, lw=2.6)
    arms = [ax.plot([], [], [], color=WHITE, lw=3)[0] for _ in range(2)]
    rotors = [ax.plot([], [], [], color=WHITE, lw=1.4, alpha=0.9)[0] for _ in range(4)]
    nose, = ax.plot([], [], [], color=RED, lw=3)

    def draw(k: int, azimuth: float) -> None:
        motors, rotor_rings, nose_tip = frame_geometry(att[k], pos[k])
        for line, (a, b) in zip(arms, ((0, 2), (1, 3))):
            line.set_data_3d(*np.stack([motors[a], motors[b]]).T)
        for line, ring in zip(rotors, rotor_rings):
            line.set_data_3d(*ring.T)
        nose.set_data_3d(*np.stack([pos[k], nose_tip]).T)
        trail.set_data_3d(*pos[: k + 1].T)
        ax.view_init(elev=22, azim=azimuth)

    output = REPO / "docs/_static/videos/hero_loop.mp4"
    poster = REPO / "docs/_static/figures/hero_poster.png"
    writer = FFMpegWriter(fps=FPS, codec="libx264",
                          extra_args=["-pix_fmt", "yuv420p", "-crf", "24", "-preset", "slow",
                                      "-movflags", "+faststart", "-an"])
    azimuths = np.linspace(-70, -40, len(indices))
    with writer.saving(fig, str(output), dpi=100):
        for k, azimuth in zip(indices, azimuths):
            draw(int(k), azimuth)
            writer.grab_frame(facecolor=TEAL)
        for _ in range(FPS):                       # short hold before the loop restarts
            writer.grab_frame(facecolor=TEAL)
    fig.savefig(poster, dpi=100, facecolor=TEAL)
    print(f"Saved {output} and {poster}")


if __name__ == "__main__":
    main()
