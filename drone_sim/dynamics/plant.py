"""Nonlinear rigid-body model of an X-configured quadcopter.

Frames and state
----------------
World coordinates are **NED** (North-East-Down) and body coordinates are
**FRD** (Forward-Right-Down). Gravity therefore points along world ``+z``.
The 12-element state vector is::

    [x, y, z, vx, vy, vz, phi, theta, psi, wx, wy, wz]

* ``x, y, z`` - position in the NED world frame (m).
* ``vx, vy, vz`` - velocity in the NED world frame (m/s).
* ``phi, theta, psi`` - ZYX Euler angles roll, pitch, yaw (rad).
* ``wx, wy, wz`` - body angular velocity in FRD axes (rad/s).

The attitude is integrated as Euler angles, so the model is singular at
pitch ``±90°`` (gimbal lock); see :func:`DronePlant.state_derivative`.

Motor layout (seen from above, nose up)::

    Motor 1 (CW)    Motor 2 (CCW)
              \\    /
               nose
              /    \\
    Motor 4 (CCW)   Motor 3 (CW)

Example:
    >>> import numpy as np
    >>> config = DroneConfig(mass=1.0, inertia=np.diag([0.02, 0.02, 0.04]),
    ...                      arm_length=0.25, drag_coefficient=0.0,
    ...                      thrust_coefficient=3e-5, yaw_torque_coefficient=1e-6)
    >>> plant = DronePlant(config, np.zeros(12))
    >>> new_state = plant.step(config.mass * GRAVITY_MPS2, np.zeros(3), 0.01)
    >>> bool(np.allclose(new_state, 0.0))  # hover thrust keeps it still
    True
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from .quaternions import Quaternion

#: Gravitational acceleration used by the plant and controllers (m/s²).
GRAVITY_MPS2: float = 9.81


@dataclass
class DroneConfig:
    """Physical parameters of the quadcopter.

    Raises:
        ValueError: If mass, arm length, thrust or yaw-torque coefficient is
            not positive, or the motor speed limit is not finite and positive.
    """

    #: Total mass (kg).
    mass: float
    #: 3×3 inertia matrix about the body FRD axes (kg·m²). Only the diagonal is used.
    inertia: np.ndarray
    #: Distance from the centre to each motor (m).
    arm_length: float
    #: Linear translational drag ``k_d`` (N per m/s). Zero disables drag.
    drag_coefficient: float
    #: Rotor thrust coefficient ``k_t`` (N per (rad/s)²); each motor produces
    #: ``k_t ω²`` of thrust.
    thrust_coefficient: float
    #: Rotor drag-torque coefficient ``k_b`` (N·m per (rad/s)²); each motor
    #: produces ``k_b ω²`` of yaw torque.
    yaw_torque_coefficient: float
    #: Upper bound on each motor speed (rad/s).
    max_motor_speed_rad_s: float = 1000.0

    def __post_init__(self) -> None:
        if (self.mass <= 0 or self.arm_length <= 0
                or self.thrust_coefficient <= 0 or self.yaw_torque_coefficient <= 0):
            raise ValueError("mass, arm_length, thrust_coefficient and "
                             "yaw_torque_coefficient must be positive")
        if not np.isfinite(self.max_motor_speed_rad_s) or self.max_motor_speed_rad_s <= 0:
            raise ValueError("max_motor_speed_rad_s must be finite and positive")


def rotation_x(phi: float) -> np.ndarray:
    """Return the right-handed rotation matrix about x (roll).

    Args:
        phi: Roll angle (rad).
    """
    return np.array([[1, 0, 0],
                     [0, np.cos(phi), -np.sin(phi)],
                     [0, np.sin(phi), np.cos(phi)]])


def rotation_y(theta: float) -> np.ndarray:
    """Return the right-handed rotation matrix about y (pitch).

    Args:
        theta: Pitch angle (rad).
    """
    return np.array([[np.cos(theta), 0, np.sin(theta)],
                     [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])


def rotation_z(psi: float) -> np.ndarray:
    """Return the right-handed rotation matrix about z (yaw).

    Args:
        psi: Yaw angle (rad).
    """
    return np.array([[np.cos(psi), -np.sin(psi), 0],
                     [np.sin(psi), np.cos(psi), 0],
                     [0, 0, 1]])


def body_to_world_rotation(phi: float, theta: float, psi: float) -> np.ndarray:
    """Return ``R = Rz(psi) Ry(theta) Rx(phi)``, mapping FRD body to NED world.

    Args:
        phi: Roll (rad).
        theta: Pitch (rad).
        psi: Yaw (rad).

    Returns:
        3×3 rotation matrix. ``R @ v_body`` gives the vector in world axes.

    Example:
        >>> import numpy as np
        >>> R = body_to_world_rotation(0.0, 0.0, np.pi / 2)
        >>> np.round(R @ [1.0, 0.0, 0.0], 12).tolist()  # nose points east
        [0.0, 1.0, 0.0]
    """
    return rotation_z(psi) @ rotation_y(theta) @ rotation_x(phi)


def world_to_body_rotation(phi: float, theta: float, psi: float) -> np.ndarray:
    """Return ``Rᵀ``, mapping NED world vectors to FRD body axes.

    Args:
        phi: Roll (rad).
        theta: Pitch (rad).
        psi: Yaw (rad).
    """
    return body_to_world_rotation(phi, theta, psi).T


def euler_to_quaternion(phi: float, theta: float, psi: float) -> Quaternion:
    """Convert ZYX Euler angles to a unit quaternion.

    Args:
        phi: Roll (rad).
        theta: Pitch (rad).
        psi: Yaw (rad).

    Returns:
        Unit quaternion of the same body-to-world rotation as
        :func:`body_to_world_rotation`.

    Example:
        >>> q = euler_to_quaternion(0.1, -0.2, 0.3)
        >>> [round(float(a), 12) for a in quaternion_to_euler(q)]
        [0.1, -0.2, 0.3]
    """
    cos_roll, sin_roll = np.cos(phi / 2), np.sin(phi / 2)
    cos_pitch, sin_pitch = np.cos(theta / 2), np.sin(theta / 2)
    cos_yaw, sin_yaw = np.cos(psi / 2), np.sin(psi / 2)

    w = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw
    x = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw
    y = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw
    z = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw

    return Quaternion(w, x, y, z).normalize()


def quaternion_to_euler(q: Quaternion) -> np.ndarray:
    """Convert a unit quaternion to ZYX Euler angles.

    Pitch is recovered with ``arcsin``, so at pitch ``±90°`` roll and yaw are no
    longer separately defined (gimbal lock); only their sum or difference is.

    Args:
        q: Unit quaternion.

    Returns:
        ``[phi, theta, psi]`` (rad), with ``theta`` in ``[-π/2, π/2]``.
    """
    w, x, y, z = q.w, q.x, q.y, q.z

    phi = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    theta = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    psi = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    return np.array([phi, theta, psi])


def euler_rate_matrix(phi: float, theta: float) -> np.ndarray:
    """Return the matrix ``E`` with ``[φ̇, θ̇, ψ̇] = E [wx, wy, wz]``.

    Args:
        phi: Roll (rad).
        theta: Pitch (rad). ``E`` contains ``tan θ`` and ``1/cos θ`` and is
            undefined at ``θ = ±π/2``.

    Returns:
        3×3 matrix mapping FRD body rates to ZYX Euler-angle rates.
    """
    return np.array([[1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                     [0, np.cos(phi), -np.sin(phi)],
                     [0, np.sin(phi) / np.cos(theta), np.cos(phi) / np.cos(theta)]])


def rk4_step(derivative: Callable[[float, np.ndarray], np.ndarray],
             t: float, y: np.ndarray, h: float) -> np.ndarray:
    """Advance ``y' = f(t, y)`` by one classical fourth-order Runge-Kutta step.

    Args:
        derivative: Function ``f(t, y)`` returning ``dy/dt``.
        t: Current time (s).
        y: Current state.
        h: Step size (s).

    Returns:
        State estimate at ``t + h`` with local error ``O(h⁵)``.

    Example:
        >>> import numpy as np
        >>> y1 = rk4_step(lambda t, y: y, 0.0, np.array([1.0]), 0.1)  # y' = y
        >>> round(float(y1[0]), 6), round(float(np.exp(0.1)), 6)
        (1.105171, 1.105171)
    """
    k1 = derivative(t, y)
    k2 = derivative(t + h/2, y + (h/2) * k1)
    k3 = derivative(t + h/2, y + (h/2) * k2)
    k4 = derivative(t + h, y + h * k3)

    return y + (h / 6) * (k1 + 2*k2 + 2*k3 + k4)


class DronePlant:
    """Quadcopter rigid body integrated with RK4.

    Args:
        config: Physical parameters.
        initial_state: 12-element state vector (see module docstring).

    Attributes:
        config: Physical parameters.
        position: NED position (m).
        velocity: NED velocity (m/s).
        euler_angles: ``[phi, theta, psi]`` (rad).
        angular_velocity: FRD body rates ``[wx, wy, wz]`` (rad/s).
    """

    def __init__(self, config: DroneConfig, initial_state: ArrayLike) -> None:
        initial_state = np.asarray(initial_state, dtype=float)
        self.config = config
        self.position = initial_state[0:3]
        self.velocity = initial_state[3:6]
        self.euler_angles = initial_state[6:9]
        self.angular_velocity = initial_state[9:12]

    @property
    def state(self) -> np.ndarray:
        """Current 12-element state vector (a new array)."""
        return np.concatenate([self.position, self.velocity,
                               self.euler_angles, self.angular_velocity])

    def _mixer_matrix(self) -> np.ndarray:
        """Return ``M`` with ``[T, τx, τy, τz] = M [ω1², ω2², ω3², ω4²]``."""
        kt = self.config.thrust_coefficient
        kb = self.config.yaw_torque_coefficient
        arm = self.config.arm_length / np.sqrt(2.0)  # moment arm of an X frame
        return np.array([[kt, kt, kt, kt],
                         [arm*kt, -arm*kt, -arm*kt, arm*kt],
                         [arm*kt, arm*kt, -arm*kt, -arm*kt],
                         [-kb, kb, -kb, kb]])

    def motor_mixing(self, motor_speeds: ArrayLike) -> tuple[float, np.ndarray]:
        """Convert four motor speeds to collective thrust and body torques.

        Args:
            motor_speeds: ``[ω1, ω2, ω3, ω4]`` (rad/s).

        Returns:
            ``(thrust, torques)``: collective thrust (N) along body ``-z`` and
            ``[τ_roll, τ_pitch, τ_yaw]`` about the FRD axes (N·m).
        """
        speeds_squared = np.asarray(motor_speeds, dtype=float) ** 2
        thrust, *torques = self._mixer_matrix() @ speeds_squared
        return float(thrust), np.array(torques)

    def current_state_derivative(self, thrust: float,
                                 torques: ArrayLike) -> np.ndarray:
        """Return ``dx/dt`` at the plant's current state.

        Args:
            thrust: Collective thrust (N).
            torques: FRD body torques (N·m).
        """
        return self.state_derivative(self.state, thrust, torques)

    def state_derivative(self, state: ArrayLike, thrust: float,
                         torques: ArrayLike) -> np.ndarray:
        """Return ``dx/dt`` of the nonlinear model at an arbitrary state.

        Translational dynamics (NED)::

            v̇ = (R [0, 0, -T] - k_d v) / m + [0, 0, g]

        Rotational dynamics (Euler's equations, diagonal inertia)::

            Euler-angle rates = E(φ, θ) ω
            I ω̇ = τ - ω × (I ω)

        Args:
            state: 12-element state vector.
            thrust: Collective thrust (N).
            torques: FRD body torques ``[τx, τy, τz]`` (N·m).

        Returns:
            12-element derivative, in the same order as ``state``.

        Raises:
            ValueError: If ``|cos θ| < 1e-6``. The Euler-rate matrix is
                singular at pitch ``±90°`` (gimbal lock).
        """
        mass = self.config.mass
        _, _, _, vx, vy, vz, phi, theta, psi, wx, wy, wz = state
        if abs(np.cos(theta)) < 1e-6:
            raise ValueError("plant Euler-angle dynamics are singular near pitch +/- 90 degrees")

        thrust_world = body_to_world_rotation(phi, theta, psi) @ np.array([0.0, 0.0, -thrust])
        gravity_world = np.array([0.0, 0.0, GRAVITY_MPS2])
        drag_world = -self.config.drag_coefficient * np.array([vx, vy, vz])
        acceleration = (thrust_world + drag_world) / mass + gravity_world

        euler_rates = euler_rate_matrix(phi, theta) @ np.array([wx, wy, wz])

        ixx, iyy, izz = np.diag(self.config.inertia)
        tau_x, tau_y, tau_z = torques
        angular_acceleration = np.array([
            ((iyy - izz) * wy * wz + tau_x) / ixx,
            ((izz - ixx) * wx * wz + tau_y) / iyy,
            ((ixx - iyy) * wx * wy + tau_z) / izz,
        ])

        return np.concatenate([[vx, vy, vz], acceleration,
                               euler_rates, angular_acceleration])

    def step(self, thrust: float, torques: ArrayLike, dt: float) -> np.ndarray:
        """Advance the plant by ``dt`` with RK4, holding the inputs constant.

        Args:
            thrust: Collective thrust (N).
            torques: FRD body torques (N·m).
            dt: Time step (s).

        Returns:
            The new 12-element state. The plant's attributes are updated too.

        Raises:
            ValueError: If ``dt`` is not positive.
        """
        if dt <= 0:
            raise ValueError("dt must be positive")

        def derivative(_t: float, y: np.ndarray) -> np.ndarray:
            return self.state_derivative(y, thrust, torques)

        new_state = rk4_step(derivative, 0.0, self.state, dt)

        self.position = new_state[0:3]
        self.velocity = new_state[3:6]
        self.euler_angles = new_state[6:9]
        self.angular_velocity = new_state[9:12]

        return new_state

    def inverse_mixing(self, thrust: float, torques: ArrayLike) -> np.ndarray:
        """Solve for the motor speeds that produce a thrust and torque request.

        Negative squared speeds are clipped to zero and speeds are capped at
        ``max_motor_speed_rad_s``, so the result may not reproduce an
        infeasible request exactly. Use :meth:`allocate_control` for a
        saturation-aware allocation.

        Args:
            thrust: Collective thrust (N).
            torques: FRD body torques (N·m).

        Returns:
            ``[ω1, ω2, ω3, ω4]`` (rad/s).
        """
        tau_roll, tau_pitch, tau_yaw = torques
        speeds_squared = np.linalg.solve(
            self._mixer_matrix(), np.array([thrust, tau_roll, tau_pitch, tau_yaw]))
        speeds_squared = np.clip(speeds_squared, 0.0, self.config.max_motor_speed_rad_s ** 2)
        return np.sqrt(speeds_squared)

    def allocate_control(self, thrust: float,
                         torques: ArrayLike) -> tuple[float, np.ndarray, np.ndarray]:
        """Turn a thrust/torque request into achievable motor speeds.

        Collective thrust is preserved when feasible. If the requested torque
        is infeasible, its magnitude is scaled uniformly to the largest value
        that keeps every motor speed within ``[0, max_motor_speed_rad_s]``.

        Args:
            thrust: Requested collective thrust (N).
            torques: Requested FRD body torques (N·m).

        Returns:
            ``(thrust, torques, motor_speeds)`` that will actually be applied:
            thrust (N), torques (N·m), and the four motor speeds (rad/s).
        """
        max_speed_squared = self.config.max_motor_speed_rad_s ** 2
        max_force = self.config.thrust_coefficient * max_speed_squared
        thrust = float(np.clip(thrust, 0.0, 4.0 * max_force))
        torques = np.asarray(torques, dtype=float).reshape(3)
        inverse_mixer = np.linalg.inv(self._mixer_matrix())
        collective_speeds_squared = inverse_mixer @ np.array([thrust, 0.0, 0.0, 0.0])
        torque_speeds_squared = inverse_mixer @ np.array([0.0, *torques])
        # The zero-torque solution loads all motors equally and is feasible
        # after clipping thrust. Find the largest torque scale analytically.
        scale = 1.0
        for base, delta in zip(collective_speeds_squared, torque_speeds_squared):
            if delta > 0:
                scale = min(scale, (max_speed_squared - base) / delta)
            elif delta < 0:
                scale = min(scale, base / -delta)
        scale = float(np.clip(scale, 0.0, 1.0))
        speeds_squared = np.clip(collective_speeds_squared + scale * torque_speeds_squared,
                                 0.0, max_speed_squared)
        motor_speeds = np.sqrt(speeds_squared)
        actual_thrust, actual_torques = self.motor_mixing(motor_speeds)
        return actual_thrust, actual_torques, motor_speeds
