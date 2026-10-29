"""Seven-second offline flight demo: plant → IMU model → filters → plots.

Data flow::

    figure-eight reference ─► controller (PID or LQR) ─► motor allocation
          ▲                                                    │
          └──────────── true plant state (NED/FRD) ◄── DronePlant (RK4)
                                   │
                    true rates + specific force (converted to up/FLU)
                                   ▼
                        MPU6050Simulator (noise)
                                   ▼
                  MadgwickMARG and IMUEKF (offline, open loop)

The controller uses the plant's **true** state, and the filters run afterwards
without affecting control. Both filters start from the known initial pose (a
simulation reference, not something an IMU can observe). During powered
flight the accelerometer measures specific force, not gravity, so the gravity
tilt correction is disabled (``gravity_valid=False``). The attitude estimates
therefore come from gyro integration alone and show raw drift.

Run with ``python -m drone_sim``.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from ..control.lqr import LQRController
from ..control.pid import CascadedController
from ..dynamics.frames import (
    filter_position_to_plot,
    ned_frd_quaternion_to_up_flu,
    ned_frd_vector_to_up_flu,
    ned_position_to_plot,
    up_flu_quaternion_to_ned_frd,
)
from ..dynamics.plant import (
    GRAVITY_MPS2,
    DroneConfig,
    DronePlant,
    body_to_world_rotation,
    euler_to_quaternion,
)
from ..estimation.ekf import IMUEKF
from ..estimation.madgwick import MadgwickMARG
from ..sensors.imu_simulator import (
    MPU6050_ACCEL_DEFAULT,
    MPU6050_GYRO_DEFAULT,
    MPU6050Config,
    MPU6050Simulator,
    plot_imu_data,
)

#: Simulation log returned by :func:`run_pipeline`.
SimulationLog = dict[str, Any]


def figure_eight_reference(t: float, duration_s: float) -> tuple[np.ndarray, np.ndarray]:
    """Return the target position and velocity of a climbing figure-eight.

    The path is ``x = 1 + 1.5 sin(ωt)``, ``y = -0.5 + 0.8 sin(2ωt)``, with altitude
    rising linearly from 2.0 m to 3.5 m, where ``ω = 2π / duration``.

    Args:
        t: Time (s).
        duration_s: Duration of one loop (s).

    Returns:
        ``(position, velocity)``: ``[x, y, altitude]`` (m) and its time
        derivative (m/s). Altitude is positive up.

    Example:
        >>> position, velocity = figure_eight_reference(0.0, 7.0)
        >>> position.tolist()
        [1.0, -0.5, 2.0]
    """
    progress = np.clip(t / duration_s, 0.0, 1.0)
    omega = 2.0 * np.pi / duration_s
    phase = omega * t
    position = np.array([1.0 + 1.5*np.sin(phase), -0.5 + 0.8*np.sin(2*phase),
                         2.0 + 1.5*progress])
    velocity = np.array([1.5*omega*np.cos(phase), 1.6*omega*np.cos(2*phase),
                         1.5/duration_s])
    return position, velocity


def wrapped_angle_error(estimate: np.ndarray, truth: np.ndarray) -> np.ndarray:
    """Return ``estimate - truth`` wrapped into ``[-π, π)`` (rad)."""
    return (np.asarray(estimate) - np.asarray(truth) + np.pi) % (2*np.pi) - np.pi


def _specific_force_frd(plant: DronePlant, thrust: float, torques: np.ndarray) -> np.ndarray:
    """Return the true specific force ``Rᵀ(a - g)`` in FRD body axes (m/s²)."""
    acceleration_ned = plant.current_state_derivative(thrust, torques)[3:6]
    gravity_ned = np.array([0.0, 0.0, GRAVITY_MPS2])
    return body_to_world_rotation(*plant.euler_angles).T @ (acceleration_ned - gravity_ned)


def _ned_euler_from_filter_quaternion(q_filter: np.ndarray) -> np.ndarray:
    """Convert a filter-frame quaternion to plant-frame ``[φ, θ, ψ]`` (rad)."""
    return MadgwickMARG.quaternion_to_euler(up_flu_quaternion_to_ned_frd(q_filter))


def run_pipeline(duration_s: float = 7.0, dt: float = 1.0 / 500, seed: int = 0,
                 controller_type: str = "pid") -> SimulationLog:
    """Fly the figure-eight, simulate the IMU, and run both filters.

    Args:
        duration_s: Simulated time (s). Must be a whole multiple of ``dt``.
        dt: Plant, controller and IMU step (s).
        seed: IMU noise seed.
        controller_type: ``"pid"`` (cascaded PID) or ``"lqr"``.

    Returns:
        Dictionary of time series. Positions are plotting coordinates
        ``(x, y, altitude)``, angles are plant ZYX Euler angles in rad, and
        arrays have ``n + 1`` rows unless noted:

        * ``t``, ``sensor_t`` — state and IMU times (s); ``control_t`` — the
          ``n`` command times (s).
        * ``pos``, ``pos_des`` — true and reference position (m).
        * ``att_true``, ``att_est`` — true and Madgwick attitude.
        * ``ekf_pos``, ``ekf_velocity``, ``ekf_att`` — EKF estimates.
        * ``rate_true`` — true FRD body rates (rad/s).
        * ``thrust`` (N), ``torques`` (N·m), ``motor_speeds`` (rad/s) — ``n`` rows.
        * ``gyro_meas`` (rad/s), ``accel_meas`` (m/s²), ``specific_force_true``
          (m/s²) — FLU body axes.
        * ``controller_type`` and ``metadata`` (frame and assumption notes).

    Raises:
        ValueError: For non-positive or non-finite ``duration_s``/``dt``, a
            duration that is not a multiple of ``dt``, or an unknown controller.

    Example:
        >>> log = run_pipeline(duration_s=0.1, dt=0.01)
        >>> log["pos"].shape
        (11, 3)
    """
    duration_s, dt = float(duration_s), float(dt)
    if not np.isfinite(duration_s) or duration_s <= 0:
        raise ValueError("duration_s must be finite and positive")
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be finite and positive")
    controller_type = str(controller_type).lower()
    if controller_type not in {"pid", "lqr"}:
        raise ValueError("controller_type must be 'pid' or 'lqr'")

    config = DroneConfig(mass=1.2, inertia=np.diag([0.02, 0.02, 0.04]), arm_length=0.25,
                         drag_coefficient=0.01, thrust_coefficient=3e-5,
                         yaw_torque_coefficient=1e-6)
    initial_state = np.zeros(12)
    initial_state[:3] = [1.0, -0.5, -2.0]       # NED: 2 m above the origin
    initial_state[6:9] = [0.3, -0.2, 0.5]       # deliberately tilted start (rad)
    initial_state[9:12] = [0.1, -0.1, 0.05]     # initial spin (rad/s)
    plant = DronePlant(config, initial_state.copy())
    if controller_type == "lqr":
        controller = LQRController(config)
    else:
        controller = CascadedController(config, outer_rate_hz=50, attitude_rate_hz=1.0/dt)

    n_steps = int(round(duration_s / dt))
    if not np.isclose(n_steps * dt, duration_s, rtol=0.0, atol=1e-10):
        raise ValueError("duration_s must be an integer multiple of dt")
    if n_steps < 1:
        raise ValueError("duration_s must contain at least one simulation step")
    t_log = np.arange(n_steps + 1, dtype=float) * dt
    pos_log = np.zeros((n_steps + 1, 3))
    pos_des_log = np.zeros((n_steps + 1, 3))
    att_true_log = np.zeros((n_steps + 1, 3))
    rate_true_log = np.zeros((n_steps + 1, 3))
    specific_force_true_log = np.zeros((n_steps + 1, 3))
    thrust_log = np.zeros(n_steps)
    torques_log = np.zeros((n_steps, 3))
    motor_speeds_log = np.zeros((n_steps, 4))
    target_yaw = 0.0

    def log_state(index: int) -> None:
        pos_log[index] = ned_position_to_plot(plant.position)
        att_true_log[index] = plant.euler_angles
        rate_true_log[index] = plant.angular_velocity
        pos_des_log[index] = figure_eight_reference(t_log[index], duration_s)[0]

    log_state(0)
    for i in range(n_steps):
        target_position, target_velocity = figure_eight_reference(t_log[i], duration_s)
        if controller_type == "lqr":
            reference = np.zeros(12)
            reference[:2] = target_position[:2]
            reference[2] = -target_position[2]          # altitude → NED down
            reference[3:5] = target_velocity[:2]
            reference[5] = -target_velocity[2]
            reference[8] = target_yaw
            command = controller.control(plant.state, reference)
            thrust, torques = command[0], command[1:4]
        else:
            thrust, torques = controller.update(target_position, target_yaw, plant, dt)
        thrust, torques, motor_speeds = plant.allocate_control(thrust, torques)
        thrust_log[i] = thrust
        torques_log[i] = torques
        motor_speeds_log[i] = motor_speeds

        if i == 0:
            specific_force_true_log[0] = ned_frd_vector_to_up_flu(
                _specific_force_frd(plant, thrust, torques))
        plant.step(thrust, torques, dt)
        log_state(i + 1)
        # Sample at the post-step state, using the command held over the step.
        specific_force_true_log[i + 1] = ned_frd_vector_to_up_flu(
            _specific_force_frd(plant, thrust, torques))

    gyro_true_up_flu = np.array([ned_frd_vector_to_up_flu(rate) for rate in rate_true_log])
    sample_rate_hz = 1.0 / dt
    imu = MPU6050Simulator(MPU6050Config(
        sample_rate_hz=sample_rate_hz, gyro_params=MPU6050_GYRO_DEFAULT,
        accel_params=MPU6050_ACCEL_DEFAULT, seed=seed))
    sensor_t, gyro_meas, accel_meas = imu.simulate(
        gyro_true_up_flu, specific_force_true_log, t_log[-1])

    # Seed both filters with the known initial pose (a simulation reference).
    initial_filter_q = ned_frd_quaternion_to_up_flu(
        euler_to_quaternion(*initial_state[6:9]).to_array())
    madgwick = MadgwickMARG(initial_quaternion=initial_filter_q, zeta=0.0,
                            sample_rate_hz=sample_rate_hz)
    ekf = IMUEKF(gravity=GRAVITY_MPS2,
                 initial_position=ned_frd_vector_to_up_flu(initial_state[:3]),
                 initial_quaternion=initial_filter_q)
    att_est_log = np.zeros((n_steps + 1, 3))
    ekf_pos_log = np.zeros((n_steps + 1, 3))
    ekf_velocity_log = np.zeros((n_steps + 1, 3))
    ekf_att_log = np.zeros((n_steps + 1, 3))
    att_est_log[0] = _ned_euler_from_filter_quaternion(madgwick.q)
    ekf_pos_log[0] = filter_position_to_plot(ekf.position)
    ekf_velocity_log[0] = filter_position_to_plot(ekf.velocity)
    ekf_att_log[0] = _ned_euler_from_filter_quaternion(ekf.quaternion)
    for i in range(1, n_steps + 1):
        q_madgwick = madgwick.update(gyro_meas[i], accel_meas[i], dt, gravity_valid=False)
        att_est_log[i] = _ned_euler_from_filter_quaternion(q_madgwick)
        ekf_position, ekf_velocity, _ = ekf.update(gyro_meas[i], accel_meas[i], dt,
                                                   gravity_valid=False)
        ekf_pos_log[i] = filter_position_to_plot(ekf_position)
        ekf_velocity_log[i] = filter_position_to_plot(ekf_velocity)
        ekf_att_log[i] = _ned_euler_from_filter_quaternion(ekf.quaternion)

    return {
        "t": t_log, "sensor_t": sensor_t, "control_t": t_log[:-1],
        "pos": pos_log, "pos_des": pos_des_log,
        "att_true": att_true_log, "att_est": att_est_log,
        "ekf_pos": ekf_pos_log, "ekf_velocity": ekf_velocity_log,
        "ekf_att": ekf_att_log, "rate_true": rate_true_log,
        "thrust": thrust_log, "torques": torques_log,
        "motor_speeds": motor_speeds_log, "gyro_meas": gyro_meas,
        "accel_meas": accel_meas, "specific_force_true": specific_force_true_log,
        "controller_type": controller_type,
        "metadata": {
            "plant_frame": "NED world / FRD body",
            "gravity_mps2": GRAVITY_MPS2,
            "filter_frame": "up-positive world / FLU body; S=diag(1,-1,-1)",
            "initial_pose_reference": "known simulation initial pose, converted to filter frame",
            "gravity_observation_valid": False,
            "gravity_gate_assumption": "powered flight acceleration is not treated as gravity; "
                                       "gyro-only attitude propagation",
            "control_feedback": "plant truth; filters run offline and do not control plant",
            "specific_force": "R_body_to_NED.T @ (plant_acceleration_NED - [0,0,g]), "
                              "transformed to filter body axes",
        },
    }


def plot_results(log: SimulationLog) -> list:
    """Build the six result figures.

    Args:
        log: Output of :func:`run_pipeline`.

    Returns:
        list[matplotlib.figure.Figure]: The figures, in this order: 3D path,
        position tracking, EKF position, attitude truth vs Madgwick, attitude
        error, and raw IMU stream.
    """
    import matplotlib.pyplot as plt

    t = log["t"]
    fig_path = plt.figure(figsize=(7, 6))
    ax = fig_path.add_subplot(projection="3d")
    ax.plot(*log["pos"].T, color="tab:blue", label="actual")
    ax.plot(*log["pos_des"].T, color="tab:red", linestyle="--", label="expected")
    ax.scatter(*log["pos"][0], color="green", s=40, label="start")
    ax.scatter(*log["pos"][-1], color="purple", s=45, label="end")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("altitude [m]")
    ax.set_title(f"Ascending Figure-Eight Flight ({log['controller_type']} controller)")
    ax.legend()
    fig_path.tight_layout()

    fig_tracking, axes_tracking = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    for i, label in enumerate(["x [m]", "y [m]", "altitude [m]"]):
        axes_tracking[i].plot(t, log["pos"][:, i], label="actual")
        axes_tracking[i].plot(t, log["pos_des"][:, i], color="tab:red", ls="--",
                              label="expected")
        axes_tracking[i].set_ylabel(label)
        axes_tracking[i].grid(alpha=0.3)
        axes_tracking[i].legend(loc="upper right")
    axes_tracking[-1].set_xlabel("time (s)")
    axes_tracking[-1].ticklabel_format(axis="y", style="plain", useOffset=False)
    fig_tracking.suptitle("Position Tracking")
    fig_tracking.tight_layout()

    fig_ekf, axes_ekf = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    for i, label in enumerate(["x", "y", "altitude"]):
        axes_ekf[i].plot(t, log["ekf_pos"][:, i],
                         label="estimate from known initial pose", alpha=0.85)
        axes_ekf[i].set_ylabel(f"{label} [m]")
        axes_ekf[i].grid(alpha=0.3)
        axes_ekf[i].legend(loc="upper right")
    axes_ekf[-1].set_xlabel("time (s)")
    axes_ekf[-1].ticklabel_format(axis="y", style="plain", useOffset=False)
    fig_ekf.suptitle("IMU-only EKF Position (estimate from known initial pose; "
                     "drifts without external reference)")
    fig_ekf.tight_layout()

    fig_attitude, axes_attitude = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    for i, label in enumerate(["roll (phi)", "pitch (theta)", "yaw (psi)"]):
        axes_attitude[i].plot(t, np.degrees(log["att_true"][:, i]), label="truth",
                              color="tab:blue")
        axes_attitude[i].plot(t, np.degrees(log["att_est"][:, i]),
                              label="Madgwick estimate", color="tab:orange", alpha=0.8)
        axes_attitude[i].set_ylabel(f"{label} (deg)")
        axes_attitude[i].grid(alpha=0.3)
        axes_attitude[i].legend()
    axes_attitude[-1].set_xlabel("time (s)")
    fig_attitude.suptitle("Attitude: Truth vs Madgwick Estimate")
    fig_attitude.tight_layout()

    fig_error, ax_error = plt.subplots(figsize=(9, 4))
    error_deg = np.degrees(wrapped_angle_error(log["att_est"], log["att_true"]))
    for i, name in enumerate(["roll", "pitch", "yaw"]):
        ax_error.plot(t, error_deg[:, i], label=name)
    ax_error.set_xlabel("time (s)")
    ax_error.set_ylabel("estimation error (deg)")
    ax_error.set_title("Madgwick Attitude Estimation Error")
    ax_error.grid(alpha=0.3)
    ax_error.legend()
    fig_error.tight_layout()

    fig_imu = plot_imu_data(log["sensor_t"], log["gyro_meas"], log["accel_meas"],
                            title="Simulated MPU6050 IMU stream")
    return [fig_path, fig_tracking, fig_ekf, fig_attitude, fig_error, fig_imu]


def arrange_plot_windows(figures: list) -> None:
    """Tile Matplotlib GUI windows in a grid on the screen (Tk backend only).

    Args:
        figures: Figures whose windows to arrange. Does nothing for
            non-GUI backends.
    """
    if not figures:
        return
    first_window = getattr(figures[0].canvas.manager, "window", None)
    if first_window is None or not hasattr(first_window, "winfo_screenwidth"):
        return
    first_window.update_idletasks()
    width, height = first_window.winfo_screenwidth(), first_window.winfo_screenheight()
    columns = min(3, len(figures))
    rows = (len(figures) + columns - 1) // columns
    for index, figure in enumerate(figures):
        window = getattr(figure.canvas.manager, "window", None)
        if window is not None and hasattr(window, "wm_geometry"):
            x = index % columns * (width // columns)
            y = index // columns * (height // rows)
            window.wm_geometry(f"{width // columns}x{height // rows}+{x}+{y}")
    first_window.update_idletasks()
    for figure in figures:
        figure.tight_layout()
        figure.canvas.draw_idle()


def main() -> None:
    """Command-line entry point: run the 7 s PID demo and show the plots."""
    import matplotlib.pyplot as plt

    duration_s, dt = 7.0, 1.0/500
    print(f"Running {duration_s:g} seconds of simulation at {1.0/dt:g} Hz...")
    print("Expected path: one figure-eight loop with a steady climb")
    print("  altitude rises from 2.0 m to 3.5 m")
    print("  it ends above its starting point")
    log = run_pipeline(duration_s=duration_s, dt=dt, seed=0, controller_type="pid")
    final_error = np.degrees(wrapped_angle_error(log["att_est"][-1], log["att_true"][-1]))
    print(f"Final position (x, y, alt): {np.round(log['pos'][-1], 3)} m")
    print(f"Final expected position (x, y, alt): {np.round(log['pos_des'][-1], 3)} m")
    ekf_displacement = log["ekf_pos"][-1] - log["ekf_pos"][0]
    print("EKF displacement from its known initial pose (x, y, altitude): "
          f"{np.round(ekf_displacement, 3)} m (drifts without external position data)")
    print(f"Final attitude estimation error (roll, pitch, yaw): {np.round(final_error, 3)} deg")
    print("Simulation finished. Opening plots...")
    figures = plot_results(log)
    arrange_plot_windows(figures)
    plt.show()


if __name__ == "__main__":
    main()
