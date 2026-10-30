"""Open-loop RK4 flight with a scripted thrust/torque sequence.

No controller is involved. The drone climbs, rolls, pitches and descends
according to :func:`scripted_maneuver`, which shows the raw plant response.
:func:`run` writes the state plots, the 3D path and the gimbal-lock figure to
one PDF.

Run ``python -m drone_sim.visualization.open_loop``.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from ..dynamics.plant import GRAVITY_MPS2, DroneConfig, DronePlant, rk4_step

#: Names of the 12 state components, in state-vector order.
STATE_COLUMNS = ["x", "y", "z", "vx", "vy", "vz",
                 "phi", "theta", "psi", "wx", "wy", "wz"]

InputFunction = Callable[[float, DroneConfig], tuple[float, np.ndarray]]


def scripted_maneuver(t: float, config: DroneConfig) -> tuple[float, np.ndarray]:
    """Return a hand-written thrust/torque schedule.

    The schedule is: climb with a roll doublet (0-2 s), hover with pitch and
    roll doublets (2-5 s), coast (5-5.5 s), then descend with combined
    doublets (5.5-9 s).

    Args:
        t: Time (s).
        config: Drone parameters (for hover thrust ``m·g``).

    Returns:
        ``(thrust, torques)``: thrust (N) and FRD torques (N·m).
    """
    hover = config.mass * GRAVITY_MPS2
    thrust = hover
    torques = np.zeros(3)

    if 0.0 <= t < 2.0:
        thrust = 1.2 * hover
        if 0.2 <= t < 0.8:
            torques[0] = 0.004
        elif 0.8 <= t < 1.4:
            torques[0] = -0.004
    elif 2.0 <= t < 5.0:
        if 2.2 <= t < 3.0:
            torques[1] = -0.004
        elif 3.0 <= t < 3.8:
            torques[1] = 0.004
        if 3.8 <= t < 4.4:
            torques[0] = -0.004
        elif 4.4 <= t < 5.0:
            torques[0] = 0.004
    elif 5.5 <= t < 9.0:
        thrust = 0.84 * hover
        if 5.5 <= t < 6.2:
            torques[1] = 0.004
        if 6.2 <= t < 6.9:
            torques[1] = -0.004
        if 5.5 <= t < 6.5:
            torques[0] = -0.006
        elif 6.5 <= t < 7.5:
            torques[0] = 0.006
    return thrust, torques


def simulate_trajectory(config: DroneConfig, initial_state: np.ndarray, t_start: float,
                        t_end: float, dt: float,
                        input_function: InputFunction = scripted_maneuver):
    """Integrate the plant open-loop with RK4.

    Args:
        config: Drone parameters.
        initial_state: 12-element state.
        t_start: Start time (s).
        t_end: End time (s).
        dt: Step (s).
        input_function: ``f(t, config) -> (thrust, torques)``.

    Returns:
        pandas.DataFrame: A ``time`` column plus the :data:`STATE_COLUMNS`.
    """
    import pandas as pd

    plant = DronePlant(config, np.zeros(12))

    def derivative(t: float, state: np.ndarray) -> np.ndarray:
        thrust, torques = input_function(t, config)
        return plant.state_derivative(state, thrust, torques)

    times = [t_start]
    states = [np.asarray(initial_state, dtype=float).copy()]
    t, state = t_start, states[0].copy()
    while t < t_end:
        state = rk4_step(derivative, t, state, dt)
        t += dt
        times.append(t)
        states.append(state.copy())

    frame = pd.DataFrame(states, columns=STATE_COLUMNS)
    frame.insert(loc=0, column="time", value=times)
    return frame


def plot_state(frame):
    """Plot position, velocity, Euler angles and body rates against time.

    Args:
        frame (pandas.DataFrame): Output of :func:`simulate_trajectory`.

    Returns:
        tuple[matplotlib.figure.Figure, numpy.ndarray]: Figure and 2×2 axes.
    """
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(10, 10))
    time = frame["time"]
    axes[0, 0].plot(time, frame["x"], label="x")
    axes[0, 0].plot(time, frame["y"], label="y")
    axes[0, 0].plot(time, -frame["z"], label="altitude")
    axes[0, 0].set_title("Position (m)")
    for name in ("vx", "vy", "vz"):
        axes[0, 1].plot(time, frame[name], label=name)
    axes[0, 1].set_title("Velocity, NED (m/s)")
    for name in ("phi", "theta", "psi"):
        axes[1, 0].plot(time, frame[name], label=name)
    axes[1, 0].set_title("Euler angles (rad)")
    for name in ("wx", "wy", "wz"):
        axes[1, 1].plot(time, frame[name], label=name)
    axes[1, 1].set_title("Body rates (rad/s)")
    for ax in axes.flat:
        ax.set_xticks(np.arange(0, time.iloc[-1], 1))
        ax.legend()
        ax.grid(True)
    axes[1, 0].set_xlabel("time (s)")
    axes[1, 1].set_xlabel("time (s)")
    fig.suptitle("Open-loop state")
    return fig, axes


def plot_trajectory(frame):
    """Plot the open-loop 3D path (altitude up).

    Returns:
        tuple[matplotlib.figure.Figure, matplotlib.axes.Axes]: Figure and 3D axes.
    """
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(projection="3d")
    ax.plot(frame["x"], frame["y"], -frame["z"], alpha=0.8)
    ax.scatter(frame["x"].iloc[0], frame["y"].iloc[0], -frame["z"].iloc[0], label="start")
    ax.scatter(frame["x"].iloc[-1], frame["y"].iloc[-1], -frame["z"].iloc[-1], label="end")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("altitude (m)")
    ax.legend()
    fig.suptitle("Open-loop 3D path")
    return fig, ax


def run(output_path: str = "outputs/visualizer_image.pdf", duration_s: float = 10.0,
        dt: float = 0.02) -> None:
    """Simulate the scripted maneuver and save all figures to one PDF.

    Args:
        output_path: PDF destination.
        duration_s: Simulated time (s).
        dt: RK4 step (s).
    """
    from matplotlib.backends.backend_pdf import PdfPages

    from .gimbal_lock import plot_gimbal_lock

    config = DroneConfig(mass=2.0, inertia=np.diag([0.01, 0.01, 0.02]), arm_length=0.2,
                         drag_coefficient=0.0, thrust_coefficient=3e-5,
                         yaw_torque_coefficient=1e-6)
    frame = simulate_trajectory(config, np.zeros(12), 0.0, duration_s, dt)
    figures = [plot_state(frame)[0], plot_trajectory(frame)[0], plot_gimbal_lock()[0]]
    with PdfPages(output_path) as pdf:
        for figure in figures:
            pdf.savefig(figure)
    print(f"Saved {output_path}")


if __name__ == "__main__":
    run()
