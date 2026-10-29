"""Noise model of the MPU6050 6-axis IMU.

Each axis of each sensor is modelled as::

    measured = true + constant_bias + gauss_markov_bias + white_noise

* **White noise** has density ``N`` (unit/√Hz). Sampled at ``fs`` it has a
  per-sample standard deviation ``N·√fs``.
* **Bias instability** is a first-order Gauss–Markov (Ornstein–Uhlenbeck)
  process with stationary standard deviation ``B`` and correlation time ``τ_c``.
  It wanders slowly, so integrating a gyro reading makes the angle drift.

The default parameters were measured on real MPU6050 units (Gonzalez & Dabove,
2019, Table 2).

Run ``python -m drone_sim.sensors.imu_simulator`` to save a 10-second static
recording plot to ``outputs/mpu6050_simulated_data.png``.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import ArrayLike
from scipy.signal import lfilter

#: Gravity used for the default static accelerometer reading (m/s²).
GRAVITY_MPS2 = 9.81


@dataclass
class AxisNoiseParams:
    """Noise parameters of one sensor axis."""

    #: White-noise density ``N`` (sensor unit per √Hz).
    noise_density: float
    #: Stationary std ``B`` of the Gauss–Markov bias (sensor unit).
    bias_std: float
    #: Bias correlation time ``τ_c`` (s).
    correlation_time_s: float


#: Measured MPU6050 gyro parameters per axis [X, Y, Z] (rad/s/√Hz, rad/s, s).
MPU6050_GYRO_DEFAULT = [
    AxisNoiseParams(noise_density=6.625e-5, bias_std=1.674e-5, correlation_time_s=900.0),
    AxisNoiseParams(noise_density=5.934e-5, bias_std=2.301e-5, correlation_time_s=200.0),
    AxisNoiseParams(noise_density=6.050e-5, bias_std=1.462e-5, correlation_time_s=200.0),
]
#: Measured MPU6050 accelerometer parameters per axis [X, Y, Z]
#: ((m/s²)/√Hz, m/s², s).
MPU6050_ACCEL_DEFAULT = [
    AxisNoiseParams(noise_density=1.156e-3, bias_std=3.703e-4, correlation_time_s=30.0),
    AxisNoiseParams(noise_density=1.252e-3, bias_std=2.501e-4, correlation_time_s=300.0),
    AxisNoiseParams(noise_density=1.820e-3, bias_std=5.058e-4, correlation_time_s=200.0),
]


def gauss_markov_bias(n_samples: int, dt: float, bias_std: float,
                      correlation_time_s: float, rng: np.random.Generator) -> np.ndarray:
    """Sample a first-order Gauss–Markov bias sequence.

    The discrete process is ``b[k+1] = φ b[k] + q w[k]`` with
    ``φ = exp(-dt/τ_c)`` and ``q = B √(1 - φ²)``, so its stationary std is ``B``.
    A burn-in of ``5 τ_c`` is discarded so the sequence starts in steady state.

    Args:
        n_samples: Number of samples to return.
        dt: Sample period (s).
        bias_std: Stationary std ``B``. Zero or negative gives an all-zero bias.
        correlation_time_s: Correlation time ``τ_c`` (s).
        rng: Random generator.

    Returns:
        Array of shape ``(n_samples,)``.

    Example:
        >>> rng = np.random.default_rng(0)
        >>> bias = gauss_markov_bias(200_000, 0.01, 2.0, 1.0, rng)
        >>> round(float(bias.std()), 1)
        2.0
    """
    if bias_std <= 0:
        return np.zeros(n_samples)
    phi = np.exp(-dt / correlation_time_s)
    drive_std = bias_std * np.sqrt(1.0 - phi ** 2)
    burn_in = int(5 * correlation_time_s / dt)
    white = rng.standard_normal(burn_in + n_samples)
    return lfilter([drive_std], [1.0, -phi], white)[burn_in:]


@dataclass
class MPU6050Config:
    """Settings for :class:`MPU6050Simulator`."""

    #: Output data rate (Hz).
    sample_rate_hz: float = 100.0
    #: Per-axis gyro noise parameters ``[X, Y, Z]``.
    gyro_params: list[AxisNoiseParams] = field(default_factory=lambda: MPU6050_GYRO_DEFAULT)
    #: Per-axis accelerometer noise parameters ``[X, Y, Z]``.
    accel_params: list[AxisNoiseParams] = field(default_factory=lambda: MPU6050_ACCEL_DEFAULT)
    #: Fixed gyro offset (rad/s).
    gyro_constant_bias_rad_s: np.ndarray = field(default_factory=lambda: np.zeros(3))
    #: Fixed accelerometer offset (m/s²).
    accel_constant_bias_mps2: np.ndarray = field(default_factory=lambda: np.zeros(3))
    #: Random seed. ``None`` gives a different recording each run.
    seed: int | None = None


class MPU6050Simulator:
    """Add MPU6050-like noise to true angular rates and specific forces.

    Args:
        config: Simulator settings.

    Example:
        >>> imu = MPU6050Simulator(MPU6050Config(sample_rate_hz=100.0, seed=1))
        >>> t, gyro, accel = imu.simulate_static(duration_s=1.0)
        >>> gyro.shape, accel.shape
        ((100, 3), (100, 3))
    """

    def __init__(self, config: MPU6050Config) -> None:
        self.config = config
        self.rng = np.random.default_rng(self.config.seed)

    def simulate(self, true_gyro_rad_s: ArrayLike, true_accel_mps2: ArrayLike,
                 duration_s: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return noisy measurements of the given true signals.

        Args:
            true_gyro_rad_s: True body rates (rad/s), either a constant ``(3,)``
                vector or an ``(n, 3)`` time series.
            true_accel_mps2: True specific force (m/s²), with the same shape
                rules as the rates.
            duration_s: Length (s). Used only for constant inputs; a time
                series sets the length itself.

        Returns:
            ``(t, gyro, accel)``: sample times (s) and ``(n, 3)`` measurement
            arrays (rad/s and m/s²).
        """
        sample_rate = self.config.sample_rate_hz
        dt = 1.0 / sample_rate

        true_gyro_rad_s = np.asarray(true_gyro_rad_s)
        true_accel_mps2 = np.asarray(true_accel_mps2)
        if true_gyro_rad_s.ndim == 1:
            n_samples = int(round(duration_s * sample_rate))
            true_gyro_rad_s = np.tile(true_gyro_rad_s, (n_samples, 1))
            true_accel_mps2 = np.tile(true_accel_mps2, (n_samples, 1))
        else:
            n_samples = true_gyro_rad_s.shape[0]

        t = np.arange(n_samples) * dt
        gyro_measured = np.empty((n_samples, 3))
        accel_measured = np.empty((n_samples, 3))

        for axis in range(3):
            gyro_param = self.config.gyro_params[axis]
            gyro_white = (gyro_param.noise_density * np.sqrt(sample_rate)
                          * self.rng.standard_normal(n_samples))
            gyro_bias = gauss_markov_bias(n_samples, dt, gyro_param.bias_std,
                                          gyro_param.correlation_time_s, self.rng)
            gyro_measured[:, axis] = (true_gyro_rad_s[:, axis]
                                      + self.config.gyro_constant_bias_rad_s[axis]
                                      + gyro_white + gyro_bias)

            accel_param = self.config.accel_params[axis]
            accel_white = (accel_param.noise_density * np.sqrt(sample_rate)
                           * self.rng.standard_normal(n_samples))
            accel_bias = gauss_markov_bias(n_samples, dt, accel_param.bias_std,
                                           accel_param.correlation_time_s, self.rng)
            accel_measured[:, axis] = (true_accel_mps2[:, axis]
                                       + self.config.accel_constant_bias_mps2[axis]
                                       + accel_bias + accel_white)

        return t, gyro_measured, accel_measured

    def simulate_static(self, duration_s: float,
                        level_accel_mps2: ArrayLike = (0.0, 0.0, GRAVITY_MPS2)
                        ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Simulate a sensor lying still and level.

        Args:
            duration_s: Length (s).
            level_accel_mps2: True specific force at rest (m/s²), z-up.

        Returns:
            Same as :meth:`simulate`.
        """
        return self.simulate(true_gyro_rad_s=np.zeros(3),
                             true_accel_mps2=np.array(level_accel_mps2),
                             duration_s=duration_s)


def plot_imu_data(t: np.ndarray, gyro: np.ndarray, accel: np.ndarray,
                  title: str = "MPU6050 simulated data"):
    """Plot gyro (rad/s) and accelerometer (m/s²) time series per axis.

    Args:
        t: Sample times (s).
        gyro: ``(n, 3)`` gyro samples (rad/s).
        accel: ``(n, 3)`` accelerometer samples (m/s²).
        title: Figure title.

    Returns:
        matplotlib.figure.Figure: A 3×2 grid (axes × sensors).
    """
    import matplotlib.pyplot as plt

    axis_labels = ["X", "Y", "Z"]
    fig, axes = plt.subplots(3, 2, figsize=(10, 8), sharex=True)
    for i in range(3):
        axes[i, 0].plot(t, gyro[:, i], color="C0")
        axes[i, 0].set_ylabel(f"gyro {axis_labels[i]}\n(rad/s)")
        axes[i, 0].grid(alpha=0.3)
        axes[i, 1].plot(t, accel[:, i], color="C1")
        axes[i, 1].set_ylabel(f"accel {axis_labels[i]}\n(m/s^2)")
        axes[i, 1].grid(alpha=0.3)
    axes[-1, 0].set_xlabel("time (s)")
    axes[-1, 1].set_xlabel("time (s)")
    fig.suptitle(title)
    fig.tight_layout()
    return fig


if __name__ == "__main__":
    simulator = MPU6050Simulator(MPU6050Config(sample_rate_hz=100.0, seed=42))
    times, gyro_samples, accel_samples = simulator.simulate_static(duration_s=10.0)
    figure = plot_imu_data(times, gyro_samples, accel_samples,
                           title="MPU6050 simulated static data (10 s)")
    figure.savefig("outputs/mpu6050_simulated_data.png", dpi=150)
