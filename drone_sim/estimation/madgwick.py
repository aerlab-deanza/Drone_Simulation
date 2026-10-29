"""Madgwick gradient-descent attitude filter (IMU and MARG variants).

The filter blends two estimates of the quaternion rate:

* **Gyro integration**, ``q̇_ω = ½ q ⊗ [0, ω]``. This is accurate over short
  times, but bias and noise make it drift.
* **Gradient correction**, ``-β ∇f / ‖∇f‖``. This is a gradient-descent step
  that rotates ``q`` so the predicted gravity (and magnetic field) directions
  match the accelerometer (and magnetometer). It is drift-free but noisy, and
  it is wrong while the body accelerates.

The update is ``q ← normalize(q + (q̇_ω - β ∇f/‖∇f‖) dt)``. ``β`` is set from
the gyro error, ``β = √(3/4) ω̃``.

Frames: the filter estimates the rotation from the **FLU body** frame to a
**z-up world** frame. A level, stationary accelerometer reads about
``[0, 0, +g]``. Quaternions are scalar-first ``[w, x, y, z]``.

Reference:
    S. O. H. Madgwick, A. J. L. Harrison and R. Vaidyanathan, "Estimation of
    IMU and MARG orientation using a gradient descent algorithm", IEEE ICORR
    2011.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike

from ..dynamics.quaternions import Quaternion

#: MPU6050 gyro white-noise density per axis (rad/s/√Hz), Gonzalez & Dabove 2019.
MPU6050_GYRO_NOISE_DENSITY = np.array([6.625e-5, 5.934e-5, 6.050e-5])
#: MPU6050 gyro bias-instability standard deviation per axis (rad/s).
MPU6050_GYRO_BIAS_STD = np.array([1.674e-5, 2.301e-5, 1.462e-5])


def default_beta(sample_rate_hz: float = 100.0) -> float:
    """Return Madgwick's ``β = √(3/4) ω̃`` for the MPU6050 noise model.

    ``ω̃`` is the mean per-axis gyro error, combining the white noise sampled at
    ``sample_rate_hz`` (``N·√fs``) with the bias standard deviation.

    Args:
        sample_rate_hz: IMU sample rate (Hz).

    Returns:
        Gradient step gain ``β`` (rad/s).

    Example:
        >>> round(default_beta(100.0), 6)
        0.000537
    """
    gyro_white_std = MPU6050_GYRO_NOISE_DENSITY * np.sqrt(sample_rate_hz)
    gyro_axis_error = np.sqrt(gyro_white_std ** 2 + MPU6050_GYRO_BIAS_STD ** 2)
    return float(np.sqrt(3.0 / 4.0) * np.mean(gyro_axis_error))


def quaternion_product(q: ArrayLike, r: ArrayLike) -> np.ndarray:
    """Return the Hamilton product ``q ⊗ r`` of two ``[w, x, y, z]`` arrays."""
    return (Quaternion.from_array(q) * Quaternion.from_array(r)).to_array()


def quaternion_conjugate(q: ArrayLike) -> np.ndarray:
    """Return the conjugate ``[w, -x, -y, -z]`` of a ``[w, x, y, z]`` array."""
    return Quaternion.from_array(q).conjugate().to_array()


def normalize_quaternion(q: ArrayLike) -> np.ndarray:
    """Return a unit-length copy of a ``[w, x, y, z]`` array."""
    return Quaternion.from_array(q).normalize().to_array()


class MadgwickMARG:
    """Madgwick attitude filter using gyro, accelerometer and optional magnetometer.

    Args:
        beta: Gradient step gain (rad/s). Defaults to :func:`default_beta`.
        zeta: Gyro-bias learning gain (rad/s²). ``0`` disables bias estimation.
        initial_quaternion: Initial attitude ``[w, x, y, z]``. Defaults to
            identity.
        sample_rate_hz: IMU rate (Hz), used only for the default ``beta``.

    Attributes:
        q: Current attitude estimate, unit ``[w, x, y, z]`` (FLU body → z-up
            world).
        gyro_bias: Estimated gyro bias (rad/s), FLU body axes.
        bx: Horizontal component of the world magnetic-field reference.
        bz: Vertical component of the world magnetic-field reference.

    Example:
        >>> filt = MadgwickMARG(zeta=0.0)
        >>> for _ in range(100):  # 0.5 rad/s about z for 1 s
        ...     q = filt.update([0.0, 0.0, 0.5], [0.0, 0.0, 9.81], dt=0.01)
        >>> round(float(filt.quaternion_to_euler(q)[2]), 3)
        0.5
    """

    def __init__(self, beta: float | None = None, zeta: float = 5e-5,
                 initial_quaternion: ArrayLike | None = None,
                 sample_rate_hz: float = 100.0) -> None:
        if beta is None:
            beta = default_beta(sample_rate_hz)
        self.beta = float(beta)
        self.zeta = float(zeta)

        if initial_quaternion is None:
            initial_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.q = normalize_quaternion(np.array(initial_quaternion, dtype=float))

        # Earth-frame magnetic reference [bx, 0, bz]; refined every MARG update.
        self.bx = 1.0
        self.bz = 0.0

        self.gyro_bias = np.zeros(3, dtype=float)

    @staticmethod
    def objective_jacobian(q: ArrayLike, bx: float, bz: float) -> np.ndarray:
        """Return the 6×4 Jacobian of :meth:`objective_function` w.r.t. ``q``.

        Rows 0–2 are the gravity terms and rows 3–5 the magnetic terms.

        Args:
            q: Attitude ``[w, x, y, z]``.
            bx: Horizontal magnetic reference.
            bz: Vertical magnetic reference.
        """
        q1, q2, q3, q4 = q
        two_bx = 2.0 * bx
        two_bz = 2.0 * bz
        return np.array([
            [-2.0*q3, 2.0*q4, -2.0*q1, 2.0*q2],
            [2.0*q2, 2.0*q1, 2.0*q4, 2.0*q3],
            [0.0, -4.0*q2, -4.0*q3, 0.0],
            [-two_bz*q3, two_bz*q4, -4.0*bx*q3 - two_bz*q1, -4.0*bx*q4 + two_bz*q2],
            [-two_bx*q4 + two_bz*q2, two_bx*q3 + two_bz*q1,
             two_bx*q2 + two_bz*q4, -two_bx*q1 + two_bz*q3],
            [two_bx*q3, two_bx*q4 - 4.0*bz*q2, two_bx*q1 - 4.0*bz*q3, two_bx*q2],
        ])

    @staticmethod
    def objective_function(q: ArrayLike, bx: float, bz: float,
                           accel_direction: ArrayLike,
                           mag_direction: ArrayLike) -> tuple[float, ...]:
        """Return the residual between predicted and measured directions.

        ``f1..f3`` are world "up" rotated into the body frame minus the
        normalized accelerometer reading. ``f4..f6`` are the field
        ``[bx, 0, bz]`` rotated into the body frame minus the normalized
        magnetometer reading.

        Args:
            q: Attitude ``[w, x, y, z]``.
            bx: Horizontal magnetic reference.
            bz: Vertical magnetic reference.
            accel_direction: Unit accelerometer vector (body FLU).
            mag_direction: Unit magnetometer vector (body FLU).

        Returns:
            ``(f1, f2, f3, f4, f5, f6)``.
        """
        ax, ay, az = accel_direction
        mx, my, mz = mag_direction
        q1, q2, q3, q4 = q
        two_bx = 2.0 * bx
        two_bz = 2.0 * bz

        f1 = 2.0*q2*q4 - 2.0*q1*q3 - ax
        f2 = 2.0*q1*q2 + 2.0*q3*q4 - ay
        f3 = 1.0 - 2.0*q2*q2 - 2.0*q3*q3 - az
        f4 = two_bx*(0.5 - q3*q3 - q4*q4) + two_bz*(q2*q4 - q1*q3) - mx
        f5 = two_bx*(q2*q3 - q1*q4) + two_bz*(q1*q2 + q3*q4) - my
        f6 = two_bx*(q1*q3 + q2*q4) + two_bz*(0.5 - q2*q2 - q3*q3) - mz
        return f1, f2, f3, f4, f5, f6

    def integrate_gyro_only(self, gyro: ArrayLike, dt: float) -> np.ndarray:
        """Propagate the attitude with the gyro alone (no correction).

        Used when a reference vector is unusable: its norm is zero, or the
        caller marked the accelerometer as not measuring gravity.

        Args:
            gyro: Body rates (rad/s), FLU.
            dt: Time step (s).

        Returns:
            Updated attitude ``[w, x, y, z]``.
        """
        gyro = np.asarray(gyro, dtype=float)
        if self.zeta != 0:
            gyro = gyro - self.gyro_bias
        q_dot = 0.5 * quaternion_product(self.q, np.array([0.0, *gyro]))
        self.q = normalize_quaternion(self.q + q_dot * dt)
        return self.q

    def update_magnetic_reference(self, mag_direction: ArrayLike) -> None:
        """Re-estimate ``bx, bz`` from a body-frame magnetometer direction.

        The measurement is rotated into the world frame as ``h = q m q*``. The
        horizontal part is then folded onto the x axis:
        ``bx = √(hx² + hy²)``, ``bz = hz``. This keeps the magnetometer from
        affecting roll and pitch.
        """
        h = quaternion_product(quaternion_product(self.q, np.array([0.0, *mag_direction])),
                               quaternion_conjugate(self.q))
        self.bx = float(np.hypot(h[1], h[2]))
        self.bz = float(h[3])

    def update(self, gyro: ArrayLike, accel: ArrayLike, dt: float,
               mag: ArrayLike | None = None, gravity_valid: bool = True) -> np.ndarray:
        """Run one filter step.

        Args:
            gyro: Body rates (rad/s), FLU.
            accel: Specific force (any unit; only its direction is used), FLU.
            dt: Time step (s). Non-positive values leave ``q`` unchanged.
            mag: Optional magnetometer reading (any unit), FLU.
            gravity_valid: Whether ``accel`` can be trusted as a gravity
                measurement. Pass False during powered or accelerated motion,
                where the accelerometer cannot tell acceleration from tilt.

        Returns:
            Updated attitude ``[w, x, y, z]``.
        """
        gyro = np.asarray(gyro, dtype=float)
        accel = np.asarray(accel, dtype=float)
        dt = float(dt)
        if dt <= 0.0:
            return self.q.copy()
        if mag is None:
            return self.update_imu(gyro, accel, dt, gravity_valid=gravity_valid)
        return self.update_marg(gyro, accel, np.asarray(mag, dtype=float), dt,
                                gravity_valid=gravity_valid)

    def _apply_correction(self, gyro: np.ndarray, gradient: np.ndarray,
                          dt: float) -> np.ndarray:
        """Learn gyro bias (if enabled) and take the fused integration step."""
        gradient = gradient / np.linalg.norm(gradient)
        if self.zeta != 0.0:
            # The gradient direction, expressed as a body rate, is the gyro
            # error; integrating it slowly estimates the bias.
            gyro_error = 2.0 * quaternion_product(quaternion_conjugate(self.q), gradient)[1:4]
            self.gyro_bias += self.zeta * gyro_error * dt
            gyro = gyro - self.gyro_bias
        q_dot_gyro = 0.5 * quaternion_product(self.q, np.array([0.0, *gyro]))
        self.q = normalize_quaternion(self.q + (q_dot_gyro - self.beta * gradient) * dt)
        return self.q

    def update_imu(self, gyro: ArrayLike, accel: ArrayLike, dt: float,
                   gravity_valid: bool = True) -> np.ndarray:
        """Run one step with gyro and accelerometer only (no heading reference).

        Args:
            gyro: Body rates (rad/s), FLU.
            accel: Specific force, FLU.
            dt: Time step (s).
            gravity_valid: See :meth:`update`.

        Returns:
            Updated attitude ``[w, x, y, z]``.
        """
        accel = np.asarray(accel, dtype=float)
        accel_norm = np.linalg.norm(accel)
        if not gravity_valid or accel_norm == 0.0:
            return self.integrate_gyro_only(gyro, dt)

        f1, f2, f3, *_ = self.objective_function(
            self.q, bx=0, bz=0, accel_direction=accel / accel_norm,
            mag_direction=[0, 0, 0])
        jacobian = self.objective_jacobian(self.q, 0, 0)[0:3]
        gradient = jacobian.T @ np.array([f1, f2, f3])
        if np.linalg.norm(gradient) == 0.0:
            return self.integrate_gyro_only(gyro, dt)
        return self._apply_correction(np.asarray(gyro, dtype=float), gradient, dt)

    def update_marg(self, gyro: ArrayLike, accel: ArrayLike, mag: ArrayLike,
                    dt: float, gravity_valid: bool = True) -> np.ndarray:
        """Run one step with gyro, accelerometer and magnetometer.

        Args:
            gyro: Body rates (rad/s), FLU.
            accel: Specific force, FLU.
            mag: Magnetometer reading, FLU.
            dt: Time step (s).
            gravity_valid: See :meth:`update`.

        Returns:
            Updated attitude ``[w, x, y, z]``.
        """
        if not gravity_valid:
            return self.integrate_gyro_only(gyro, dt)
        accel = np.asarray(accel, dtype=float)
        mag = np.asarray(mag, dtype=float)
        accel_norm = np.linalg.norm(accel)
        mag_norm = np.linalg.norm(mag)
        if accel_norm == 0.0 or mag_norm == 0.0:
            return self.integrate_gyro_only(gyro, dt)

        mag_direction = mag / mag_norm
        objective = np.array(self.objective_function(
            self.q, self.bx, self.bz,
            accel_direction=accel / accel_norm, mag_direction=mag_direction))
        gradient = self.objective_jacobian(self.q, self.bx, self.bz).T @ objective
        if np.linalg.norm(gradient) == 0:
            return self.integrate_gyro_only(gyro, dt)

        self._apply_correction(np.asarray(gyro, dtype=float), gradient, dt)
        self.update_magnetic_reference(mag_direction)
        return self.q

    @staticmethod
    def quaternion_to_euler(q: ArrayLike) -> np.ndarray:
        """Convert ``[w, x, y, z]`` to ``[roll, pitch, yaw]`` (rad), ZYX order.

        Pitch is clipped to ``[-π/2, π/2]``. Near ``±90°`` pitch, roll and yaw
        become ill-defined (gimbal lock).
        """
        qw, qx, qy, qz = q
        roll = np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
        pitch = np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
        yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return np.array([roll, pitch, yaw])
