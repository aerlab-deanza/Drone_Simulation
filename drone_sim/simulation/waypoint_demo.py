"""Twenty-second waypoint flight with the cascaded PID controller.

The drone hovers, flies to ``(3, 2, 2.2) m`` while yawing 45°, holds, and
returns. Each leg follows a minimum-jerk profile. The plant runs at 5 kHz.

Run ``python -m drone_sim.simulation.waypoint_demo`` to save
``outputs/trajectory_3d.png`` and show the 3D path.
"""

from __future__ import annotations

import numpy as np

from ..control.pid import CascadedController
from ..dynamics.plant import DroneConfig, DronePlant

#: Plant integration step (s), i.e. 5 kHz.
DT = 0.0002
#: Hover altitude at the start and end (m).
BASE_ALTITUDE = 1.5

#: ``(time s, [x, y, altitude] m, yaw rad)`` waypoints.
WAYPOINTS = [
    (0.0, np.array([0.0, 0.0, BASE_ALTITUDE]), 0.0),
    (2.0, np.array([0.0, 0.0, BASE_ALTITUDE]), 0.0),
    (8.0, np.array([3.0, 2.0, 2.2]), np.radians(45)),
    (11.0, np.array([3.0, 2.0, 2.2]), np.radians(45)),
    (17.0, np.array([0.0, 0.0, BASE_ALTITUDE]), 0.0),
    (20.0, np.array([0.0, 0.0, BASE_ALTITUDE]), 0.0),
]
#: Total simulated time (s).
SIM_TIME = WAYPOINTS[-1][0]


def make_drone_config() -> DroneConfig:
    """Return the 1 kg demo airframe (no drag)."""
    return DroneConfig(mass=1.0, inertia=np.diag([0.01, 0.01, 0.02]), arm_length=0.25,
                       drag_coefficient=0.0, thrust_coefficient=3.0e-5,
                       yaw_torque_coefficient=1.0e-6)


def minimum_jerk(progress: float) -> float:
    """Return the minimum-jerk blend ``10s³ - 15s⁴ + 6s⁵`` for ``s ∈ [0, 1]``.

    It has zero velocity and acceleration at both ends.

    Example:
        >>> minimum_jerk(0.0), minimum_jerk(0.5), minimum_jerk(1.0)
        (0.0, 0.5, 1.0)
    """
    s = float(np.clip(progress, 0.0, 1.0))
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def desired_trajectory(t: float) -> tuple[np.ndarray, float]:
    """Interpolate :data:`WAYPOINTS` at time ``t``.

    Args:
        t: Time (s).

    Returns:
        ``(target_position, target_yaw)``: ``[x, y, altitude]`` (m) and yaw
        (rad). Yaw takes the shortest way round.
    """
    if t <= WAYPOINTS[0][0]:
        return WAYPOINTS[0][1], WAYPOINTS[0][2]
    for (t0, p0, yaw0), (t1, p1, yaw1) in zip(WAYPOINTS[:-1], WAYPOINTS[1:]):
        if t0 <= t <= t1:
            blend = minimum_jerk((t - t0) / (t1 - t0) if t1 > t0 else 1.0)
            yaw_delta = np.arctan2(np.sin(yaw1 - yaw0), np.cos(yaw1 - yaw0))
            return p0 + (p1 - p0) * blend, yaw0 + yaw_delta * blend
    return WAYPOINTS[-1][1], WAYPOINTS[-1][2]


def run_simulation() -> dict[str, np.ndarray]:
    """Fly the waypoint mission.

    Returns:
        Log with keys ``t`` (s), ``pos`` and ``pos_des`` (``x, y, altitude`` m),
        ``att`` and ``att_des`` (rad), ``rate`` (rad/s), ``thrust`` (N),
        ``torques`` (N·m) and ``motor_speeds`` (rad/s).
    """
    config = make_drone_config()
    initial_state = np.zeros(12)
    initial_state[2] = -BASE_ALTITUDE   # NED: hovering 1.5 m up
    plant = DronePlant(config, initial_state)
    controller = CascadedController(config, outer_rate_hz=50, attitude_rate_hz=5000)

    n_steps = int(SIM_TIME / DT)
    log = {
        "t": np.zeros(n_steps),
        "pos": np.zeros((n_steps, 3)),
        "pos_des": np.zeros((n_steps, 3)),
        "att": np.zeros((n_steps, 3)),
        "att_des": np.zeros((n_steps, 3)),
        "rate": np.zeros((n_steps, 3)),
        "thrust": np.zeros(n_steps),
        "torques": np.zeros((n_steps, 3)),
        "motor_speeds": np.zeros((n_steps, 4)),
    }
    for i in range(n_steps):
        t = i * DT
        target_position, target_yaw = desired_trajectory(t)
        thrust, torques = controller.update(target_position, target_yaw, plant, DT)
        thrust, torques, motor_speeds = plant.allocate_control(thrust, torques)
        plant.step(thrust, torques, DT)

        x, y, z_down = plant.position
        log["t"][i] = t
        log["pos"][i] = [x, y, -z_down]
        log["pos_des"][i] = target_position
        log["att"][i] = plant.euler_angles
        log["att_des"][i] = controller.target_attitude
        log["rate"][i] = plant.angular_velocity
        log["thrust"][i] = thrust
        log["torques"][i] = torques
        log["motor_speeds"][i] = motor_speeds
    return log


def plot_trajectory_3d(log: dict[str, np.ndarray]):
    """Plot actual vs desired 3D path.

    Returns:
        matplotlib.figure.Figure: The figure.
    """
    import matplotlib.pyplot as plt

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(*log["pos"].T, label="actual", color="blue")
    ax.plot(*log["pos_des"].T, label="desired", color="red")
    ax.scatter(*log["pos"][0], color="green", s=40, label="start")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("altitude [m]")
    ax.set_title("3D Trajectory")
    ax.legend()
    fig.tight_layout()
    return fig


def main() -> None:
    """Run the mission, save the 3D plot, and show it."""
    import matplotlib.pyplot as plt

    log = run_simulation()
    output_path = "outputs/trajectory_3d.png"
    plot_trajectory_3d(log).savefig(output_path, dpi=150)
    print(f"Saved {output_path}")
    plt.show()


if __name__ == "__main__":
    main()
