"""Cascaded PID flight controller.

The controller has three nested loops, from slowest to fastest::

    target position ──► PositionController ──► target attitude + thrust
    target attitude ──► AttitudeController ──► target body rates
    target body rates ─► RateController ─────► body torques

:class:`CascadedController` runs each loop at its own rate. All loops read the
plant's true state (no estimator in the loop). The plant frames are NED world
and FRD body (see :mod:`drone_sim.dynamics.plant`).
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike

from ..dynamics.plant import GRAVITY_MPS2, DroneConfig, DronePlant


def wrap_angle(angle: float | np.ndarray) -> float | np.ndarray:
    """Wrap an angle (rad) into ``(-π, π]``.

    Example:
        >>> round(float(wrap_angle(3 * np.pi / 2)), 6)
        -1.570796
    """
    return np.arctan2(np.sin(angle), np.cos(angle))


class PID:
    """Single-axis PID controller with a clamped integrator.

    The output is ``kp·e + ki·∫e dt + kd·de/dt`` with ``e = setpoint - value``.
    The derivative term is zero on the first update, so a step setpoint does not
    produce a derivative kick on the first sample.

    Args:
        kp: Proportional gain.
        ki: Integral gain.
        kd: Derivative gain.
        setpoint: Initial target value.
        integral_limit: Bound on ``|∫e dt|`` (anti-windup).
        wrap_angle_error: If True, the error is wrapped into ``(-π, π]`` so an
            angle loop always turns the short way round.

    Attributes:
        integral: Accumulated ``∫e dt``.
        previous_error: Error from the last update.

    Example:
        >>> pid = PID(kp=2.0, ki=0.0, kd=0.0, setpoint=1.0)
        >>> float(pid.update(0.25, dt=0.01))
        1.5
    """

    def __init__(self, kp: float, ki: float, kd: float, setpoint: float,
                 integral_limit: float = np.inf, wrap_angle_error: bool = False) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self.integral = 0.0
        self.previous_error = 0.0
        self.integral_limit = float(integral_limit)
        self.wrap_angle_error = wrap_angle_error
        self.initialized = False

    def update(self, measured_value: float, dt: float) -> float:
        """Return the control output for one sample.

        Args:
            measured_value: Current value of the controlled quantity.
            dt: Time since the previous update (s).

        Returns:
            PID output, in the units implied by the gains.

        Raises:
            ValueError: If ``dt`` is not positive.
        """
        if dt <= 0:
            raise ValueError("dt must be positive")
        error = self.setpoint - measured_value
        if self.wrap_angle_error:
            error = wrap_angle(error)

        self.integral = np.clip(self.integral + error * dt,
                                -self.integral_limit, self.integral_limit)

        if not self.initialized:
            derivative = 0.0
        elif self.wrap_angle_error:
            derivative = wrap_angle(error - self.previous_error) / dt
        else:
            derivative = (error - self.previous_error) / dt

        self.previous_error = error
        self.initialized = True

        return self.kp * error + self.ki * self.integral + self.kd * derivative


class PositionController:
    """Outer loop: position and altitude errors → target attitude and thrust.

    Altitude error drives a vertical acceleration command, which becomes
    collective thrust after dividing by ``cos φ cos θ`` (tilt compensation).
    Horizontal errors drive world-frame accelerations. These are rotated into
    the heading frame and turned into small tilt angles, using the near-hover
    approximation ``a ≈ g·tan(tilt)``.

    Args:
        config: Drone parameters.
    """

    #: Largest commanded roll or pitch (rad).
    MAX_TILT = np.radians(20)
    #: Bound on the vertical acceleration command (m/s²).
    ALTITUDE_ACCEL_LIMIT = 10.0
    #: Bound on each horizontal acceleration command (m/s²).
    XY_ACCEL_LIMIT = 4.0
    #: Anti-windup bound on the altitude integral (m·s).
    ALTITUDE_INTEGRAL_LIMIT = 3.0
    #: Anti-windup bound on each horizontal integral (m·s).
    XY_INTEGRAL_LIMIT = 3.0

    def __init__(self, config: DroneConfig) -> None:
        self.config = config
        # Gains output accelerations in m/s² (before thrust/tilt mapping).
        self.pid_altitude = PID(kp=8, ki=2, kd=5, setpoint=0,
                                integral_limit=self.ALTITUDE_INTEGRAL_LIMIT)
        self.pid_x = PID(kp=10.0, ki=0.0, kd=10.0, setpoint=0,
                         integral_limit=self.XY_INTEGRAL_LIMIT)
        self.pid_y = PID(kp=10.0, ki=0.0, kd=10.0, setpoint=0,
                         integral_limit=self.XY_INTEGRAL_LIMIT)

    def update(self, target_position: ArrayLike, target_yaw: float,
               plant: DronePlant, dt: float) -> tuple[np.ndarray, float]:
        """Compute the target attitude and collective thrust.

        Args:
            target_position: ``[x, y, altitude]`` (m). Altitude is positive up.
            target_yaw: Target heading ``ψ`` (rad).
            plant: Plant whose true state is fed back.
            dt: Time since the previous outer-loop update (s).

        Returns:
            ``(target_attitude, thrust)``: ``[φ, θ, ψ]`` (rad) and collective
            thrust (N).
        """
        target_x, target_y, target_altitude = target_position
        x, y, z_down = plant.position
        altitude = -z_down
        phi, theta, psi = plant.euler_angles

        # Thrust is along body -z; only cos(phi)cos(theta) of it acts vertically.
        vertical_fraction = max(float(np.cos(phi) * np.cos(theta)), 0.1)

        self.pid_altitude.setpoint = target_altitude
        vertical_accel = np.clip(self.pid_altitude.update(altitude, dt),
                                 -self.ALTITUDE_ACCEL_LIMIT, self.ALTITUDE_ACCEL_LIMIT)
        thrust = max(0.0, plant.config.mass * (GRAVITY_MPS2 + vertical_accel) / vertical_fraction)

        self.pid_x.setpoint = target_x
        accel_x_world = np.clip(self.pid_x.update(x, dt),
                                -self.XY_ACCEL_LIMIT, self.XY_ACCEL_LIMIT)
        self.pid_y.setpoint = target_y
        accel_y_world = np.clip(self.pid_y.update(y, dt),
                                -self.XY_ACCEL_LIMIT, self.XY_ACCEL_LIMIT)

        # Rotate the world-frame request into the current heading frame.
        accel_forward = np.cos(psi) * accel_x_world + np.sin(psi) * accel_y_world
        accel_right = -np.sin(psi) * accel_x_world + np.cos(psi) * accel_y_world
        # FRD: nose-down (negative pitch) accelerates forward; right-down
        # (positive roll) accelerates right.
        target_pitch = np.clip(-np.arctan2(accel_forward, GRAVITY_MPS2),
                               -self.MAX_TILT, self.MAX_TILT)
        target_roll = np.clip(np.arctan2(accel_right, GRAVITY_MPS2),
                              -self.MAX_TILT, self.MAX_TILT)

        return np.array([target_roll, target_pitch, target_yaw]), thrust


class AttitudeController:
    """Middle loop: attitude error → target FRD body rates.

    Each Euler angle has a P controller producing an Euler-angle *rate*. The
    rates are then mapped to body rates with the inverse Euler-rate matrix.

    Args:
        config: Drone parameters (unused; kept for a uniform constructor).
    """

    def __init__(self, config: DroneConfig) -> None:
        self.pid_roll = PID(kp=6.0, ki=0.0, kd=0, setpoint=0)
        self.pid_pitch = PID(kp=6.0, ki=0.0, kd=0, setpoint=0)
        self.pid_yaw = PID(kp=6.0, ki=0.0, kd=0, setpoint=0, wrap_angle_error=True)

    def update(self, target_attitude: ArrayLike, plant: DronePlant,
               dt: float) -> np.ndarray:
        """Compute target body rates.

        Args:
            target_attitude: ``[φ, θ, ψ]`` (rad).
            plant: Plant whose true attitude is fed back.
            dt: Time since the previous attitude-loop update (s).

        Returns:
            Target FRD body rates ``[wx, wy, wz]`` (rad/s).
        """
        target_roll, target_pitch, target_yaw = target_attitude
        phi, theta, psi = plant.euler_angles

        self.pid_roll.setpoint = target_roll
        roll_rate = self.pid_roll.update(phi, dt)
        self.pid_pitch.setpoint = target_pitch
        pitch_rate = self.pid_pitch.update(theta, dt)
        self.pid_yaw.setpoint = target_yaw
        yaw_rate = self.pid_yaw.update(psi, dt)

        euler_rates_to_body_rates = np.array([
            [1.0, 0.0, -np.sin(theta)],
            [0.0, np.cos(phi), np.sin(phi) * np.cos(theta)],
            [0.0, -np.sin(phi), np.cos(phi) * np.cos(theta)],
        ])
        return euler_rates_to_body_rates @ np.array([roll_rate, pitch_rate, yaw_rate])


class RateController:
    """Inner loop: body-rate error → body torques.

    Args:
        config: Drone parameters (unused; kept for a uniform constructor).
    """

    def __init__(self, config: DroneConfig) -> None:
        self.pid_wx = PID(kp=1.0, ki=0.0, kd=0.0, setpoint=0)
        self.pid_wy = PID(kp=1.0, ki=0.0, kd=0.0, setpoint=0)
        self.pid_wz = PID(kp=1.0, ki=0.0, kd=0.0, setpoint=0)

    def update(self, target_rates: ArrayLike, plant: DronePlant,
               dt: float) -> np.ndarray:
        """Compute body torques.

        Args:
            target_rates: Target FRD body rates ``[wx, wy, wz]`` (rad/s).
            plant: Plant whose true body rates are fed back.
            dt: Time since the previous update (s).

        Returns:
            FRD body torques ``[τx, τy, τz]`` (N·m).
        """
        target_wx, target_wy, target_wz = target_rates
        wx, wy, wz = plant.angular_velocity

        self.pid_wx.setpoint = target_wx
        self.pid_wy.setpoint = target_wy
        self.pid_wz.setpoint = target_wz
        return np.array([self.pid_wx.update(wx, dt),
                         self.pid_wy.update(wy, dt),
                         self.pid_wz.update(wz, dt)])


class CascadedController:
    """Multi-rate scheduler for the position → attitude → rate loops.

    The rate loop runs on every call. The attitude and position loops run when
    their period has elapsed and receive the true elapsed time since their last
    run, so their integrators and derivatives stay correct.

    Args:
        config: Drone parameters.
        outer_rate_hz: Position-loop rate (Hz).
        attitude_rate_hz: Attitude-loop rate (Hz).

    Attributes:
        target_attitude: Latest attitude command ``[φ, θ, ψ]`` (rad).
        thrust: Latest collective-thrust command (N).
        target_rates: Latest body-rate command (rad/s).

    Example:
        >>> from drone_sim.dynamics.plant import DroneConfig, DronePlant
        >>> config = DroneConfig(1.0, np.diag([0.02, 0.02, 0.04]), 0.25, 0.0, 3e-5, 1e-6)
        >>> plant = DronePlant(config, np.zeros(12))
        >>> controller = CascadedController(config)
        >>> thrust, torques = controller.update([0.0, 0.0, 0.0], 0.0, plant, 0.002)
        >>> round(thrust, 2)  # starts at hover thrust, m·g
        9.81
    """

    def __init__(self, config: DroneConfig, outer_rate_hz: float = 50,
                 attitude_rate_hz: float = 500) -> None:
        self.position_control = PositionController(config)
        self.attitude_control = AttitudeController(config)
        self.rate_control = RateController(config)

        self.outer_period = 1.0 / outer_rate_hz
        self.attitude_period = 1.0 / attitude_rate_hz

        self._outer_phase = 0.0
        self._attitude_phase = 0.0
        self._outer_elapsed = 0.0
        self._attitude_elapsed = 0.0

        self.target_attitude = np.zeros(3)
        self.thrust = config.mass * GRAVITY_MPS2
        self.target_rates = np.zeros(3)

    def update(self, target_position: ArrayLike, target_yaw: float,
               plant: DronePlant, dt: float) -> tuple[float, np.ndarray]:
        """Advance the scheduler by ``dt`` and return the actuator command.

        Args:
            target_position: ``[x, y, altitude]`` (m), altitude positive up.
            target_yaw: Target heading (rad).
            plant: Plant whose true state is fed back.
            dt: Simulation step (s).

        Returns:
            ``(thrust, torques)``: collective thrust (N) and FRD body torques
            (N·m), before motor saturation.
        """
        self._outer_phase += dt
        self._attitude_phase += dt
        self._outer_elapsed += dt
        self._attitude_elapsed += dt

        if self._outer_phase >= self.outer_period:
            self.target_attitude, self.thrust = self.position_control.update(
                target_position, target_yaw, plant, self._outer_elapsed)
            self._outer_phase %= self.outer_period
            self._outer_elapsed = 0.0

        if self._attitude_phase >= self.attitude_period:
            self.target_rates = self.attitude_control.update(
                self.target_attitude, plant, self._attitude_elapsed)
            self._attitude_phase %= self.attitude_period
            self._attitude_elapsed = 0.0

        torques = self.rate_control.update(self.target_rates, plant, dt)
        return self.thrust, torques
