"""IMU-only extended Kalman filter (EKF) for relative pose estimation.

The 16-element state is::

    [position (3), velocity (3), quaternion w,x,y,z (4),
     gyro bias (3), accelerometer bias (3)]

**Predict** - the gyro (minus its bias) integrates the quaternion. The
accelerometer (minus its bias), rotated into the world and with gravity
removed, integrates velocity and position (strapdown mechanization).

**Update** - when the caller marks the accelerometer as a valid gravity
observation, its direction corrects roll and pitch. An optional magnetometer
corrects yaw.

Nothing observes position or velocity directly, so position is *relative* and
drifts quadratically with any acceleration error. This is the expected
behaviour of an IMU-only filter.

Frames: **FLU body** and **z-up world**. Accelerometer samples are specific
force, so a level, stationary sensor reads ``[0, 0, +g]``. Units are SI.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Sequence

import numpy as np
from numpy.typing import ArrayLike

IDENTITY_QUATERNION = np.array([1.0, 0.0, 0.0, 0.0])


def normalize_quaternion(q: ArrayLike) -> np.ndarray:
    """Return ``q / ‖q‖``, or the identity quaternion if ``‖q‖ ≈ 0``."""
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return IDENTITY_QUATERNION.copy()
    return q / norm


def rotate_vector(q: ArrayLike, vector: ArrayLike) -> np.ndarray:
    """Rotate a body-frame vector into the world frame with unit quaternion ``q``.

    Uses the expanded sandwich product
    ``v' = v + 2 q_v × (q_v × v + w v)``, which avoids building a matrix.

    Args:
        q: Unit quaternion ``[w, x, y, z]`` (body → world).
        vector: 3-vector in body axes.

    Returns:
        The vector in world axes. Pass the conjugate of ``q`` to rotate from
        world to body.

    Example:
        >>> quarter_turn_z = [np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)]
        >>> (np.round(rotate_vector(quarter_turn_z, [1.0, 0.0, 0.0]), 12) + 0.0).tolist()
        [0.0, 1.0, 0.0]
    """
    w, x, y, z = q
    vector = np.asarray(vector, dtype=float)
    q_vector = np.array([x, y, z])
    return vector + 2.0 * np.cross(q_vector, np.cross(q_vector, vector) + w * vector)


def quaternion_to_euler(q: ArrayLike) -> np.ndarray:
    """Convert ``[w, x, y, z]`` to ZYX ``[roll, pitch, yaw]`` (rad)."""
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def _conjugate(q: np.ndarray) -> np.ndarray:
    return q * np.array([1.0, -1.0, -1.0, -1.0])


class IMUEKF:
    """Estimate relative position, velocity and attitude from IMU samples.

    Args:
        gravity: Local gravity magnitude (m/s²).
        initial_position: Starting position in the filter world frame (m).
        accel_noise_std: Accelerometer white-noise std per sample (m/s²).
        gyro_noise_std: Gyro white-noise std per sample (rad/s).
        magnetic_field_world: Optional world-frame magnetic direction. If it is
            None and magnetometer samples arrive, the first sample defines the
            heading reference.
        estimate_biases: If False, the bias states are fixed at zero and
            excluded from the covariance.
        initial_quaternion: Initial attitude ``[w, x, y, z]`` (FLU → z-up).

    Attributes:
        state: 16-element state vector (see the module docstring).
        covariance: 16×16 state covariance.

    Example:
        >>> ekf = IMUEKF(estimate_biases=False)
        >>> for _ in range(100):  # still and level for 1 s
        ...     position, velocity, attitude = ekf.update(
        ...         [0.0, 0.0, 0.0], [0.0, 0.0, 9.80665], dt=0.01)
        >>> bool(np.allclose(position, 0.0)), bool(np.allclose(attitude, 0.0))
        (True, True)
    """

    #: Slice of the position states (m).
    POSITION = slice(0, 3)
    #: Slice of the velocity states (m/s).
    VELOCITY = slice(3, 6)
    #: Slice of the quaternion states, scalar first.
    QUATERNION = slice(6, 10)
    #: Slice of the gyro-bias states (rad/s).
    GYRO_BIAS = slice(10, 13)
    #: Slice of the accelerometer-bias states (m/s²).
    ACCEL_BIAS = slice(13, 16)
    #: Total number of states.
    STATE_SIZE = 16

    def __init__(self, gravity: float = 9.80665,
                 initial_position: ArrayLike = (0.0, 0.0, 0.0),
                 accel_noise_std: float = 0.15, gyro_noise_std: float = 0.02,
                 magnetic_field_world: ArrayLike | None = None,
                 estimate_biases: bool = True,
                 initial_quaternion: ArrayLike = (1.0, 0.0, 0.0, 0.0)) -> None:
        self.gravity = float(gravity)
        self.state = np.zeros(self.STATE_SIZE, dtype=float)
        self.state[self.POSITION] = np.asarray(initial_position, dtype=float)
        self.state[self.QUATERNION] = normalize_quaternion(initial_quaternion)
        self.covariance = np.diag([1.0] * 3 + [1.0] * 3 + [0.1] * 4 + [0.05] * 3 + [0.1] * 3)
        self.accel_noise_std = float(accel_noise_std)
        self.gyro_noise_std = float(gyro_noise_std)
        self.magnetic_field_world = self._unit_or_none(magnetic_field_world)
        self.estimate_biases = bool(estimate_biases)
        if not self.estimate_biases:
            self.state[self.GYRO_BIAS] = 0.0
            self.state[self.ACCEL_BIAS] = 0.0
            bias_indices = list(range(self.GYRO_BIAS.start, self.ACCEL_BIAS.stop))
            self.covariance[bias_indices, :] = 0.0
            self.covariance[:, bias_indices] = 0.0

    @property
    def position(self) -> np.ndarray:
        """Estimated position, z-up world frame (m)."""
        return self.state[self.POSITION].copy()

    @property
    def velocity(self) -> np.ndarray:
        """Estimated velocity, z-up world frame (m/s)."""
        return self.state[self.VELOCITY].copy()

    @property
    def quaternion(self) -> np.ndarray:
        """Estimated attitude ``[w, x, y, z]`` (FLU body → z-up world)."""
        return self.state[self.QUATERNION].copy()

    @property
    def attitude(self) -> np.ndarray:
        """Estimated ``[roll, pitch, yaw]`` (rad)."""
        return quaternion_to_euler(self.quaternion)

    def initialize_tilt(self, specific_force_body: ArrayLike, heading: float = 0.0) -> None:
        """Set roll and pitch from one stationary accelerometer sample.

        Gravity says nothing about heading, so the caller supplies ``heading``.
        Position is kept and velocity is reset to zero.

        Args:
            specific_force_body: Stationary accelerometer reading, FLU (any unit).
            heading: Known yaw (rad).
        """
        measured_up = self._unit_or_none(specific_force_body)
        if measured_up is None:
            return
        # In the ZYX convention, body-frame "up" is
        # [-sin(pitch), sin(roll)cos(pitch), cos(roll)cos(pitch)].
        ax, ay, az = measured_up
        roll = np.arctan2(ay, az)
        pitch = np.arctan2(-ax, np.hypot(ay, az))
        cos_roll, sin_roll = np.cos(0.5 * roll), np.sin(0.5 * roll)
        cos_pitch, sin_pitch = np.cos(0.5 * pitch), np.sin(0.5 * pitch)
        cos_yaw, sin_yaw = np.cos(0.5 * heading), np.sin(0.5 * heading)
        self.state[self.QUATERNION] = normalize_quaternion(np.array([
            cos_roll*cos_pitch*cos_yaw + sin_roll*sin_pitch*sin_yaw,
            sin_roll*cos_pitch*cos_yaw - cos_roll*sin_pitch*sin_yaw,
            cos_roll*sin_pitch*cos_yaw + sin_roll*cos_pitch*sin_yaw,
            cos_roll*cos_pitch*sin_yaw - sin_roll*sin_pitch*cos_yaw,
        ]))
        self.state[self.VELOCITY] = 0.0

    @staticmethod
    def _unit_or_none(vector: ArrayLike | None) -> np.ndarray | None:
        if vector is None:
            return None
        vector = np.asarray(vector, dtype=float)
        norm = np.linalg.norm(vector)
        return vector / norm if norm > 1e-12 else None

    @classmethod
    def transition(cls, state: np.ndarray, gyro: np.ndarray, accel: np.ndarray,
                   dt: float, gravity: float) -> np.ndarray:
        """Propagate a state by one IMU sample (strapdown mechanization).

        ``q ← normalize(q + ½ q ⊗ [0, ω - b_g] dt)``,
        ``a = R(q)(f - b_a) - [0, 0, g]``,
        ``p ← p + v dt + ½ a dt²``, ``v ← v + a dt``.

        Args:
            state: 16-element state.
            gyro: Gyro sample (rad/s), FLU.
            accel: Accelerometer specific force (m/s²), FLU.
            dt: Time step (s).
            gravity: Gravity magnitude (m/s²).

        Returns:
            Predicted 16-element state.
        """
        next_state = state.copy()
        q = normalize_quaternion(state[cls.QUATERNION])
        omega = gyro - state[cls.GYRO_BIAS]
        w, x, y, z = q
        ox, oy, oz = omega
        # q̇ = ½ q ⊗ [0, ω], expanded.
        q_dot = 0.5 * np.array([
            - x * ox - y * oy - z * oz,
            w * ox + y * oz - z * oy,
            w * oy - x * oz + z * ox,
            w * oz + x * oy - y * ox,
        ])
        q_next = normalize_quaternion(q + q_dot * dt)

        specific_force_body = accel - state[cls.ACCEL_BIAS]
        acceleration_world = rotate_vector(q, specific_force_body) + np.array([0.0, 0.0, -gravity])
        next_state[cls.POSITION] = (state[cls.POSITION] + state[cls.VELOCITY] * dt
                                    + 0.5 * acceleration_world * dt * dt)
        next_state[cls.VELOCITY] = state[cls.VELOCITY] + acceleration_world * dt
        next_state[cls.QUATERNION] = q_next
        return next_state

    @classmethod
    def gravity_measurement_model(cls, state: np.ndarray, gravity: float) -> np.ndarray:
        """Predict the accelerometer reading of a non-accelerating body.

        ``h(x) = R(q)ᵀ [0, 0, g] + b_a``: the support force, pointing world-up,
        expressed in body axes.
        """
        q = normalize_quaternion(state[cls.QUATERNION])
        world_up_in_body = rotate_vector(_conjugate(q), np.array([0.0, 0.0, gravity]))
        return world_up_in_body + state[cls.ACCEL_BIAS]

    @classmethod
    def magnetic_measurement_model(cls, state: np.ndarray,
                                   field_world: np.ndarray) -> np.ndarray:
        """Predict the body-frame magnetic direction, ``h(x) = R(q)ᵀ m_world``."""
        q = normalize_quaternion(state[cls.QUATERNION])
        return rotate_vector(_conjugate(q), field_world)

    def _numerical_jacobian(self, function: Callable[[np.ndarray], np.ndarray],
                            state: np.ndarray, output_size: int,
                            indices: Iterable[int] | None = None) -> np.ndarray:
        """Central-difference Jacobian of ``function`` w.r.t. selected states."""
        jacobian = np.zeros((output_size, self.STATE_SIZE))
        epsilon = 1e-6
        if indices is None:
            indices = range(self.STATE_SIZE)
        for index in indices:
            plus = state.copy()
            minus = state.copy()
            plus[index] += epsilon
            minus[index] -= epsilon
            jacobian[:, index] = (function(plus) - function(minus)) / (2.0 * epsilon)
        return jacobian

    def _measurement_update(self, measurement: np.ndarray,
                            model: Callable[[np.ndarray], np.ndarray],
                            noise_std: float, active_states: Sequence[int] | range) -> None:
        """Apply one EKF correction (Joseph-form covariance update).

        ``K = P Hᵀ (H P Hᵀ + R)⁻¹``, ``x ← x + K (z - h(x))``,
        ``P ← (I - K H) P (I - K H)ᵀ + K R Kᵀ``.
        """
        predicted = model(self.state)
        observation_jacobian = self._numerical_jacobian(
            model, self.state, measurement.size, active_states)
        measurement_covariance = np.eye(measurement.size) * noise_std**2
        innovation = measurement - predicted
        innovation_covariance = (observation_jacobian @ self.covariance @ observation_jacobian.T
                                 + measurement_covariance)
        gain = np.linalg.solve(innovation_covariance,
                               observation_jacobian @ self.covariance).T
        self.state += gain @ innovation

        residual_map = np.eye(self.STATE_SIZE) - gain @ observation_jacobian
        self.covariance = (residual_map @ self.covariance @ residual_map.T
                           + gain @ measurement_covariance @ gain.T)
        self._normalize_quaternion_and_covariance()

    def _normalize_quaternion_and_covariance(self) -> None:
        """Renormalize the quaternion and project its covariance consistently.

        Unit quaternions live on the sphere S³, so the radial direction is not
        observable. The covariance is mapped through the Jacobian of
        ``q ↦ q/‖q‖``, which is ``(I - q̂ q̂ᵀ)/‖q‖``.
        """
        q_raw = self.state[self.QUATERNION].copy()
        norm = np.linalg.norm(q_raw)
        if norm < 1e-12:
            self.state[self.QUATERNION] = IDENTITY_QUATERNION
            return
        q = q_raw / norm
        self.state[self.QUATERNION] = q
        jacobian = np.eye(self.STATE_SIZE)
        jacobian[self.QUATERNION, self.QUATERNION] = (np.eye(4) - np.outer(q, q)) / norm
        self.covariance = jacobian @ self.covariance @ jacobian.T
        self.covariance = 0.5 * (self.covariance + self.covariance.T)

    def update(self, gyro: ArrayLike, accel: ArrayLike, dt: float,
               mag: ArrayLike | None = None,
               gravity_valid: bool = True) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Predict with one IMU sample, then apply the available corrections.

        Args:
            gyro: Gyro sample (rad/s), FLU.
            accel: Accelerometer specific force (m/s²), FLU.
            dt: Time since the previous sample (s).
            mag: Optional magnetometer reading, FLU (any unit).
            gravity_valid: Whether ``accel`` may be used as a gravity
                observation. Even when True, samples whose norm is outside
                ``[0.75 g, 1.25 g]`` are ignored.

        Returns:
            ``(position, velocity, attitude)``: m, m/s, and
            ``[roll, pitch, yaw]`` in rad.

        Raises:
            ValueError: If the inputs have the wrong shape, are not finite, or
                ``dt`` is not positive.
        """
        gyro = np.asarray(gyro, dtype=float)
        accel = np.asarray(accel, dtype=float)
        dt = float(dt)
        if gyro.shape != (3,) or accel.shape != (3,):
            raise ValueError("gyro and accel samples must each have shape (3,)")
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        if not np.all(np.isfinite(gyro)) or not np.all(np.isfinite(accel)):
            raise ValueError("gyro and accel samples must be finite")

        prior = self.transition(self.state, gyro, accel, dt, self.gravity)
        # During fast turns, linear acceleration and vibration distort the
        # reference directions, so their measurement noise grows with the
        # rotation rate. The gyro then leads the attitude change.
        rotation_rate = np.linalg.norm(gyro - self.state[self.GYRO_BIAS])
        reference_noise_scale = 1.0 + (rotation_rate / 0.45) ** 2

        # Linear blocks are written analytically. Only the blocks that depend
        # on the quaternion and biases are differentiated numerically.
        transition_jacobian = np.eye(self.STATE_SIZE)
        transition_jacobian[self.POSITION, self.VELOCITY] = np.eye(3) * dt
        q = normalize_quaternion(self.state[self.QUATERNION])
        q_columns = range(self.QUATERNION.start, self.QUATERNION.stop)
        gyro_bias_columns = range(self.GYRO_BIAS.start, self.GYRO_BIAS.stop)

        def world_acceleration(state: np.ndarray) -> np.ndarray:
            return (rotate_vector(normalize_quaternion(state[self.QUATERNION]),
                                  accel - state[self.ACCEL_BIAS])
                    + np.array([0.0, 0.0, -self.gravity]))

        accel_jacobian = self._numerical_jacobian(world_acceleration, self.state, 3, q_columns)
        rotation = np.column_stack([rotate_vector(q, axis) for axis in np.eye(3)])
        transition_jacobian[self.POSITION, q_columns] = 0.5 * accel_jacobian[:, q_columns] * dt**2
        transition_jacobian[self.VELOCITY, q_columns] = accel_jacobian[:, q_columns] * dt
        transition_jacobian[self.POSITION, self.ACCEL_BIAS] = -0.5 * rotation * dt**2
        transition_jacobian[self.VELOCITY, self.ACCEL_BIAS] = -rotation * dt

        def quaternion_transition(state: np.ndarray) -> np.ndarray:
            return self.transition(state, gyro, accel, dt, self.gravity)[self.QUATERNION]

        quaternion_jacobian = self._numerical_jacobian(
            quaternion_transition, self.state, 4, list(q_columns) + list(gyro_bias_columns))
        transition_jacobian[self.QUATERNION, q_columns] = quaternion_jacobian[:, q_columns]
        transition_jacobian[self.QUATERNION, gyro_bias_columns] = (
            quaternion_jacobian[:, gyro_bias_columns])

        # Process noise: per-sample sensor noise mapped through the discrete
        # transition (keeping position/velocity cross terms), plus a slow
        # random walk on the biases.
        bias_random_walk = [1e-7] * 3 + [1e-6] * 3 if self.estimate_biases else [0.0] * 6
        accel_input = np.zeros((self.STATE_SIZE, 3))
        accel_input[self.POSITION, :] = 0.5 * rotation * dt**2
        accel_input[self.VELOCITY, :] = rotation * dt
        gyro_input = np.empty((self.STATE_SIZE, 3))
        epsilon = 1e-6
        for axis in range(3):
            delta = np.zeros(3)
            delta[axis] = epsilon
            gyro_input[:, axis] = (
                self.transition(self.state, gyro + delta, accel, dt, self.gravity)
                - self.transition(self.state, gyro - delta, accel, dt, self.gravity)
            ) / (2.0 * epsilon)
        process_noise = (accel_input @ (np.eye(3) * self.accel_noise_std**2) @ accel_input.T
                         + gyro_input @ (np.eye(3) * self.gyro_noise_std**2) @ gyro_input.T
                         + np.diag([0.0] * 10 + bias_random_walk) * dt)
        self.state = prior
        self.covariance = (transition_jacobian @ self.covariance @ transition_jacobian.T
                           + process_noise)
        self._normalize_quaternion_and_covariance()

        # The norm gate rejects large disturbances, but it cannot separate a
        # sustained acceleration from gravity. The caller decides validity.
        accel_norm = np.linalg.norm(accel)
        if gravity_valid and 0.75 * self.gravity <= accel_norm <= 1.25 * self.gravity:
            accel_bias_columns = (list(range(self.ACCEL_BIAS.start, self.ACCEL_BIAS.stop))
                                  if self.estimate_biases else [])
            self._measurement_update(
                accel,
                lambda state: self.gravity_measurement_model(state, self.gravity),
                self.accel_noise_std * reference_noise_scale,
                list(q_columns) + accel_bias_columns,
            )

        if mag is not None:
            mag = np.asarray(mag, dtype=float)
            if mag.shape != (3,) or not np.all(np.isfinite(mag)):
                raise ValueError("mag sample must be a finite vector with shape (3,)")
            mag_direction = self._unit_or_none(mag)
            if mag_direction is not None:
                if self.magnetic_field_world is None:
                    self.magnetic_field_world = self._unit_or_none(
                        rotate_vector(self.quaternion, mag_direction))
                self._measurement_update(
                    mag_direction,
                    lambda state: self.magnetic_measurement_model(
                        state, self.magnetic_field_world),
                    0.05 * reference_noise_scale,
                    q_columns,
                )

        return self.position, self.velocity, self.attitude
