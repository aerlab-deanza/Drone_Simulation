"""Numerical demonstration of gimbal lock in ZYX Euler angles.

Two symptoms appear as pitch ``θ`` approaches ±90°:

1. **Rate blow-up.** The Euler-rate matrix ``E(φ, θ)`` contains ``tan θ`` and
   ``1/cos θ``, so a constant, bounded body rate maps to unbounded roll and yaw
   rates.
2. **Lost degree of freedom.** At ``θ = ±90°`` the rotation matrix depends only
   on ``ψ ∓ φ``. Recovering Euler angles from the matrix is then not unique, and
   the roll/yaw split returned by a solver jumps.

Quaternions avoid both: ``q̇ = ½ q ⊗ [0, ω]`` is bounded for every attitude,
which is why the estimators in :mod:`drone_sim.estimation` use them.
"""

from __future__ import annotations

import numpy as np

from ..dynamics.plant import body_to_world_rotation, euler_rate_matrix


def euler_rates_near_vertical(
    roll_rad: float = np.deg2rad(10.0),
    body_rates_rad_s: tuple[float, float, float] = (1.0, 0.2, 0.5),
    pitch_deg_range: tuple[float, float] = (70.0, 110.0),
    gap_deg: float = 0.1,
    n_points: int = 200,
) -> dict[str, np.ndarray]:
    """Map a constant body rate to Euler-angle rates for pitch near 90°.

    Args:
        roll_rad: Fixed roll (rad).
        body_rates_rad_s: Constant FRD body rate ``[wx, wy, wz]`` (rad/s).
        pitch_deg_range: Pitch sweep ``(start, stop)`` (deg).
        gap_deg: Half-width of the excluded band around 90° (deg).
        n_points: Samples on each side of 90°.

    Returns:
        ``pitch_deg``, ``secant`` (``|1/cos θ|``) and ``euler_rates_deg_s``
        (``(n, 3)`` array of roll, pitch and yaw rates, deg/s).

    Example:
        >>> data = euler_rates_near_vertical(n_points=5)
        >>> bool(data["secant"].max() > 500)
        True
    """
    low, high = pitch_deg_range
    pitch_deg = np.concatenate([np.linspace(low, 90.0 - gap_deg, n_points),
                                np.linspace(90.0 + gap_deg, high, n_points)])
    pitch_rad = np.deg2rad(pitch_deg)
    rates = np.array([euler_rate_matrix(roll_rad, theta) @ np.asarray(body_rates_rad_s)
                      for theta in pitch_rad])
    return {"pitch_deg": pitch_deg,
            "secant": np.abs(1.0 / np.cos(pitch_rad)),
            "euler_rates_deg_s": np.rad2deg(rates)}


def recovered_euler_angles(roll_deg: float = 10.0, yaw_deg: float = 20.0,
                           pitch_deg: np.ndarray | None = None) -> dict[str, np.ndarray]:
    """Round-trip Euler angles → rotation matrix → Euler angles.

    Away from ±90° pitch the input is recovered exactly. Near it the solver
    still returns a valid rotation, but roll and yaw no longer match the input.

    Args:
        roll_deg: Input roll (deg).
        yaw_deg: Input yaw (deg).
        pitch_deg: Pitch sweep (deg). Defaults to 70–110° skipping 90° ± 0.1°.

    Returns:
        ``pitch_deg`` and ``recovered_deg``, an ``(n, 3)`` array of recovered
        ``[roll, pitch, yaw]`` (deg).
    """
    from scipy.spatial.transform import Rotation

    if pitch_deg is None:
        pitch_deg = euler_rates_near_vertical()["pitch_deg"]
    recovered = []
    for theta in np.deg2rad(pitch_deg):
        matrix = body_to_world_rotation(np.deg2rad(roll_deg), theta, np.deg2rad(yaw_deg))
        yaw, pitch, roll = Rotation.from_matrix(matrix).as_euler("ZYX")
        recovered.append([roll, pitch, yaw])
    return {"pitch_deg": np.asarray(pitch_deg), "recovered_deg": np.rad2deg(recovered)}


def plot_gimbal_lock(roll_deg: float = 10.0, yaw_deg: float = 20.0):
    """Plot both gimbal-lock symptoms in a 2×2 figure.

    Args:
        roll_deg: Input roll (deg).
        yaw_deg: Input yaw (deg).

    Returns:
        tuple[matplotlib.figure.Figure, numpy.ndarray]: Figure and axes grid.
    """
    import matplotlib.pyplot as plt

    rates = euler_rates_near_vertical(roll_rad=np.deg2rad(roll_deg))
    recovery = recovered_euler_angles(roll_deg, yaw_deg, rates["pitch_deg"])
    pitch_deg = rates["pitch_deg"]

    fig, axes = plt.subplots(2, 2, figsize=(10, 10))
    axes[0, 0].plot(pitch_deg, rates["secant"], label="|1 / cos(theta)|")
    axes[0, 0].set_title("1 / cos(theta)")
    for i, name in enumerate(["roll rate", "pitch rate", "yaw rate"]):
        axes[0, 1].plot(pitch_deg, rates["euler_rates_deg_s"][:, i], label=name)
    axes[0, 1].set_ylabel("Euler rate (deg/s)")
    axes[0, 1].set_title("Euler-angle rates for a constant body rate")

    axes[1, 0].plot(pitch_deg, np.full_like(pitch_deg, roll_deg), label="input roll")
    axes[1, 0].plot(pitch_deg, pitch_deg, label="input pitch")
    axes[1, 0].plot(pitch_deg, np.full_like(pitch_deg, yaw_deg), label="input yaw")
    axes[1, 0].set_ylabel("angle (deg)")
    axes[1, 0].set_title("Input Euler angles")
    for i, name in enumerate(["recovered roll", "recovered pitch", "recovered yaw"]):
        axes[1, 1].plot(pitch_deg, recovery["recovered_deg"][:, i], label=name)
    axes[1, 1].set_ylabel("angle (deg)")
    axes[1, 1].set_title("Euler angles recovered from the rotation matrix")

    for ax in axes.flat:
        ax.set_xlabel("input pitch (deg)")
        ax.legend(loc="lower left")
        ax.grid(True)
    fig.suptitle("Gimbal lock")
    fig.subplots_adjust(wspace=0.3)
    return fig, axes
