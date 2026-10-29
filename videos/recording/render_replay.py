"""Render a 3D replay of the offline figure-eight demo to MP4.

Run from the repository root::

    python videos/recording/render_replay.py [--controller pid|lqr] [--speed 0.5]

Writes ``docs/_static/videos/static_replay.mp4``. The left panel shows the
drone oriented by its **true** attitude, a translucent orange ghost oriented by
the **Madgwick estimate** (drawn as-is; the error is a fraction of a degree, so
the ghost nearly coincides with the true model), and trails for the true path, the reference and the
IMU-only EKF position. The right panels plot attitude-estimation error and
tracking error as the replay advances.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FFMpegWriter  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from drone_sim.dynamics.plant import body_to_world_rotation  # noqa: E402
from drone_sim.simulation.pipeline import run_pipeline, wrapped_angle_error  # noqa: E402

FPS = 30
ARM = 0.25 * 2.2          # displayed arm length (m), enlarged for visibility
ROTOR_RADIUS = 0.12 * 2.2


def frame_geometry(euler_ned: np.ndarray, center_plot: np.ndarray):
    """Return arm segments and rotor circles of an X-frame in plot coordinates."""
    rotation = body_to_world_rotation(*euler_ned)
    to_plot = np.diag([1.0, 1.0, -1.0])            # NED → (x, y, altitude)
    diagonal = ARM / np.sqrt(2.0)
    motors_body = np.array([[diagonal, -diagonal, 0], [diagonal, diagonal, 0],
                            [-diagonal, diagonal, 0], [-diagonal, -diagonal, 0]])
    motors = (to_plot @ rotation @ motors_body.T).T + center_plot
    angles = np.linspace(0, 2 * np.pi, 24)
    circle_body = np.stack([np.cos(angles), np.sin(angles), np.zeros_like(angles)], axis=1) * ROTOR_RADIUS
    rotors = [(to_plot @ rotation @ (circle_body + m).T).T + center_plot for m in motors_body]
    nose = (to_plot @ rotation @ np.array([ARM * 0.9, 0, 0])) + center_plot
    return motors, rotors, nose


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", default="pid", choices=["pid", "lqr"])
    parser.add_argument("--speed", type=float, default=0.5, help="playback speed vs real time")
    parser.add_argument("--output", default=str(REPO / "docs/_static/videos/static_replay.mp4"))
    args = parser.parse_args()

    log = run_pipeline(controller_type=args.controller)
    t = log["t"]
    sim_dt = t[1] - t[0]
    stride = max(1, int(round(args.speed / (FPS * sim_dt))))   # sim samples per video frame
    indices = np.arange(0, len(t), stride)
    attitude_error = np.degrees(wrapped_angle_error(log["att_est"], log["att_true"]))
    tracking_error = np.linalg.norm(log["pos"] - log["pos_des"], axis=1)

    plt.rcParams.update({"font.size": 11, "axes.titlesize": 12})
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    grid = fig.add_gridspec(2, 2, width_ratios=(1.45, 1.0), hspace=0.38, wspace=0.12,
                            left=0.01, right=0.97, top=0.9, bottom=0.08)
    ax3d = fig.add_subplot(grid[:, 0], projection="3d")
    ax_att = fig.add_subplot(grid[0, 1])
    ax_track = fig.add_subplot(grid[1, 1])
    fig.suptitle(f"Offline demo replay — climbing figure-eight, {args.controller.upper()} controller "
                 f"({args.speed:g}× speed)", fontsize=14)

    pos, ref, ekf = log["pos"], log["pos_des"], log["ekf_pos"]
    margin = 0.7
    ax3d.set_xlim(pos[:, 0].min() - margin, pos[:, 0].max() + margin)
    ax3d.set_ylim(pos[:, 1].min() - margin, pos[:, 1].max() + margin)
    ax3d.set_zlim(1.5, 4.0)
    ax3d.set_box_aspect((1.3, 1.0, 0.8))
    ax3d.set_xlabel("x [m]")
    ax3d.set_ylabel("y [m]")
    ax3d.set_zlabel("altitude [m]")
    ax3d.view_init(elev=24, azim=-58)
    ax3d.plot(*ref.T, "--", color="tab:red", lw=1.2, alpha=0.6, label="reference")
    ekf_trail, = ax3d.plot([], [], [], color="tab:green", lw=2.0, ls=":", label="EKF position (IMU only)")
    true_trail, = ax3d.plot([], [], [], color="tab:blue", lw=2.2, label="true path")
    arms_true = [ax3d.plot([], [], [], color="#1f2d3d", lw=3)[0] for _ in range(2)]
    rotors_true = [ax3d.plot([], [], [], color="#1f2d3d", lw=1.6)[0] for _ in range(4)]
    nose_true, = ax3d.plot([], [], [], color="tab:red", lw=3)
    arms_est = [ax3d.plot([], [], [], color="tab:orange", lw=2, alpha=0.55)[0] for _ in range(2)]
    ax3d.plot([], [], color="tab:orange", alpha=0.55, lw=2, label="Madgwick attitude (ghost)")
    ax3d.legend(loc="upper left", fontsize=9)
    clock = ax3d.text2D(0.02, 0.02, "", transform=ax3d.transAxes, family="monospace", fontsize=11)

    ax_att.set_title("Attitude estimation error (Madgwick − truth)")
    ax_att.set_xlim(0, t[-1])
    span = max(0.05, np.abs(attitude_error).max() * 1.15)
    ax_att.set_ylim(-span, span)
    ax_att.set_ylabel("deg")
    ax_att.grid(alpha=0.3)
    att_lines = [ax_att.plot([], [], label=n)[0] for n in ("roll", "pitch", "yaw")]
    ax_att.legend(loc="upper right", ncol=3, fontsize=9)

    ax_track.set_title("Position: tracking error vs EKF drift")
    ax_track.set_xlim(0, t[-1])
    ekf_error = np.linalg.norm(ekf - pos, axis=1)
    ax_track.set_ylim(0, max(tracking_error.max(), ekf_error.max()) * 1.1)
    ax_track.set_xlabel("time (s)")
    ax_track.set_ylabel("m")
    ax_track.grid(alpha=0.3)
    track_line, = ax_track.plot([], [], color="tab:blue", label="|true − reference|")
    ekf_line, = ax_track.plot([], [], color="tab:green", label="|EKF − true|")
    ax_track.legend(loc="upper right", fontsize=9)

    def draw(k: int) -> None:
        center = pos[k]
        motors, rotors, nose = frame_geometry(log["att_true"][k], center)
        for line, (a, b) in zip(arms_true, ((0, 2), (1, 3))):
            line.set_data_3d(*np.stack([motors[a], motors[b]]).T)
        for line, rotor in zip(rotors_true, rotors):
            line.set_data_3d(*rotor.T)
        nose_true.set_data_3d(*np.stack([center, nose]).T)
        est_motors, _, _ = frame_geometry(log["att_est"][k], center)
        for line, (a, b) in zip(arms_est, ((0, 2), (1, 3))):
            line.set_data_3d(*np.stack([est_motors[a], est_motors[b]]).T)
        true_trail.set_data_3d(*pos[:k + 1].T)
        ekf_trail.set_data_3d(*ekf[:k + 1].T)
        roll, pitch, yaw = np.degrees(log["att_true"][k])
        clock.set_text(f"t = {t[k]:4.2f} s   roll {roll:+6.1f}°  pitch {pitch:+6.1f}°  yaw {yaw:+6.1f}°")
        for i, line in enumerate(att_lines):
            line.set_data(t[:k + 1], attitude_error[:k + 1, i])
        track_line.set_data(t[:k + 1], tracking_error[:k + 1])
        ekf_line.set_data(t[:k + 1], ekf_error[:k + 1])

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = FFMpegWriter(fps=FPS, codec="libx264",
                          extra_args=["-pix_fmt", "yuv420p", "-crf", "22", "-preset", "slow",
                                      "-movflags", "+faststart"])
    with writer.saving(fig, str(output), dpi=100):
        draw(0)
        for _ in range(FPS):                 # hold the first frame for 1 s
            writer.grab_frame()
        for k in indices:
            draw(int(k))
            writer.grab_frame()
        draw(len(t) - 1)
        for _ in range(2 * FPS):             # hold the last frame for 2 s
            writer.grab_frame()
    print(f"Saved {output}")


if __name__ == "__main__":
    main()
