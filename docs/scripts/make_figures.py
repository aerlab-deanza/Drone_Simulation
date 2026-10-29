"""Regenerate every figure used by the documentation.

Run from the repository root::

    python docs/scripts/make_figures.py

Figures are written to ``docs/_static/figures``. Every figure comes from the
package code, so the plots in the docs always match the implementation.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
OUT = REPO_ROOT / "docs" / "_static" / "figures"

from drone_sim.estimation.madgwick import MadgwickMARG  # noqa: E402
from drone_sim.sensors.imu_simulator import (  # noqa: E402
    MPU6050_ACCEL_DEFAULT,
    MPU6050_GYRO_DEFAULT,
    MPU6050Config,
    MPU6050Simulator,
)
from drone_sim.simulation.pipeline import plot_results, run_pipeline  # noqa: E402
from drone_sim.visualization.gimbal_lock import plot_gimbal_lock  # noqa: E402


def save(fig, name: str) -> None:
    fig.savefig(OUT / name, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {name}")


def gyro_drift_figure(duration_s: float = 60.0, sample_rate_hz: float = 100.0,
                      n_runs: int = 200) -> plt.Figure:
    """Monte Carlo of gyro-only yaw drift for a still MPU6050, with √t envelope."""
    rng_seeds = np.arange(n_runs)
    n = int(duration_s * sample_rate_hz)
    t = np.arange(1, n + 1) / sample_rate_hz
    yaw = np.empty((n_runs, n))
    for run, seed in enumerate(rng_seeds):
        imu = MPU6050Simulator(MPU6050Config(sample_rate_hz=sample_rate_hz, seed=int(seed)))
        _, gyro, _ = imu.simulate_static(duration_s)
        yaw[run] = np.cumsum(gyro[:, 2]) / sample_rate_hz
    noise_density = MPU6050_GYRO_DEFAULT[2].noise_density
    bias_std = MPU6050_GYRO_DEFAULT[2].bias_std
    fig, ax = plt.subplots(figsize=(8, 4.2))
    for run in range(min(n_runs, 25)):
        ax.plot(t, np.degrees(yaw[run]), color="tab:blue", alpha=0.25, lw=0.8)
    ax.plot(t, np.degrees(yaw.std(axis=0)), color="black", lw=2, label="measured 1σ (200 runs)")
    ax.plot(t, np.degrees(noise_density * np.sqrt(t)), "--", color="tab:red", lw=2,
            label=r"white noise only: $N\sqrt{t}$")
    ax.plot(t, np.degrees(np.sqrt(noise_density**2 * t + (bias_std * t) ** 2)), ":",
            color="tab:green", lw=2, label=r"$\sqrt{N^2 t + (B t)^2}$")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("integrated yaw (deg)")
    ax.set_title("Gyro-only drift of a motionless MPU6050 (z axis)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left")
    return fig


def tilt_correction_figure(duration_s: float = 30.0, sample_rate_hz: float = 100.0) -> plt.Figure:
    """Gyro integration vs Madgwick with gravity correction, on a biased gyro."""
    dt = 1.0 / sample_rate_hz
    bias = np.array([0.01, -0.008, 0.0])  # rad/s, a deliberately poor calibration
    imu = MPU6050Simulator(MPU6050Config(sample_rate_hz=sample_rate_hz, seed=3,
                                         gyro_constant_bias_rad_s=bias))
    t, gyro, accel = imu.simulate_static(duration_s)
    gyro_only = MadgwickMARG(beta=0.0, zeta=0.0)
    fused = MadgwickMARG(beta=0.05, zeta=0.0)
    roll_only, roll_fused = [], []
    for g_sample, a_sample in zip(gyro, accel):
        roll_only.append(gyro_only.quaternion_to_euler(gyro_only.update(g_sample, a_sample, dt))[0])
        roll_fused.append(fused.quaternion_to_euler(fused.update(g_sample, a_sample, dt))[0])
    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.plot(t, np.degrees(roll_only), label="gyro integration only (β = 0)")
    ax.plot(t, np.degrees(roll_fused), label="Madgwick, gravity correction (β = 0.05)")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("roll estimate (deg)")
    ax.set_title("A 0.01 rad/s gyro bias: integration drifts, fusion stays bounded")
    ax.grid(alpha=0.3)
    ax.legend()
    return fig


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    print("Generating documentation figures...")
    save(plot_gimbal_lock()[0], "gimbal_lock.png")
    save(gyro_drift_figure(), "gyro_drift.png")
    save(tilt_correction_figure(), "tilt_correction.png")

    names = ["pipeline_path_3d.png", "pipeline_tracking.png", "pipeline_ekf_position.png",
             "pipeline_attitude.png", "pipeline_attitude_error.png", "pipeline_imu.png"]
    for figure, name in zip(plot_results(run_pipeline(controller_type="pid")), names):
        save(figure, name)
    lqr_figures = plot_results(run_pipeline(controller_type="lqr"))
    save(lqr_figures[0], "pipeline_path_3d_lqr.png")
    for figure in lqr_figures[1:]:
        plt.close(figure)

    from drone_sim.control.lqr import run_recovery_demo
    run_recovery_demo(str(OUT / "lqr_recovery.png"))
    plt.close("all")
    _ = MPU6050_ACCEL_DEFAULT  # referenced in the docs alongside the gyro table


if __name__ == "__main__":
    main()
