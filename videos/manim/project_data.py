"""Project data used by the theory videos, computed once and cached.

Every curve in the videos comes from the ``drone_sim`` package. Expensive
results (Monte Carlo runs, full pipeline simulations) are cached as ``.npz``
files in ``videos/build/data`` so re-rendering a scene is fast. Delete that
folder to recompute.
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import numpy as np

import common  # noqa: F401  (puts the repository on sys.path)
from drone_sim.estimation.madgwick import MadgwickMARG
from drone_sim.sensors.imu_simulator import (
    MPU6050_GYRO_DEFAULT,
    AxisNoiseParams,
    MPU6050Config,
    MPU6050Simulator,
)

CACHE = Path(__file__).resolve().parents[1] / "build" / "data"


def _cached(name: str, compute) -> dict[str, np.ndarray]:
    CACHE.mkdir(parents=True, exist_ok=True)
    path = CACHE / f"{name}.npz"
    if path.exists():
        return dict(np.load(path, allow_pickle=False))
    data = compute()
    np.savez_compressed(path, **data)
    return data


@lru_cache
def gyro_drift(duration_s: float = 60.0, rate_hz: float = 100.0, runs: int = 200) -> dict:
    """Integrated z-gyro angle (deg) of a motionless MPU6050, many runs."""
    def compute():
        n = int(duration_s * rate_hz)
        t = np.arange(1, n + 1) / rate_hz
        angles = np.empty((runs, n))
        for seed in range(runs):
            imu = MPU6050Simulator(MPU6050Config(sample_rate_hz=rate_hz, seed=seed))
            _, gyro, _ = imu.simulate_static(duration_s)
            angles[seed] = np.degrees(np.cumsum(gyro[:, 2]) / rate_hz)
        return {"t": t, "angles": angles,
                "N": np.array(MPU6050_GYRO_DEFAULT[2].noise_density),
                "B": np.array(MPU6050_GYRO_DEFAULT[2].bias_std)}
    return _cached(f"gyro_drift_{int(duration_s)}_{int(rate_hz)}_{runs}", compute)


@lru_cache
def pipeline_errors() -> dict:
    """Madgwick roll error (deg) for noisy vs noise-free IMU, at 500 Hz and 2 kHz."""
    def compute():
        import drone_sim.simulation.pipeline as pipeline

        def roll_error(log):
            return np.degrees(pipeline.wrapped_angle_error(log["att_est"], log["att_true"]))[:, 0]

        noisy = pipeline.run_pipeline()
        zero = [AxisNoiseParams(0.0, 0.0, 1.0)] * 3
        original = pipeline.MPU6050_GYRO_DEFAULT, pipeline.MPU6050_ACCEL_DEFAULT
        pipeline.MPU6050_GYRO_DEFAULT = pipeline.MPU6050_ACCEL_DEFAULT = zero
        try:
            clean = pipeline.run_pipeline()
            clean_fast = pipeline.run_pipeline(dt=1.0 / 2000)
        finally:
            pipeline.MPU6050_GYRO_DEFAULT, pipeline.MPU6050_ACCEL_DEFAULT = original
        return {"t": noisy["t"], "noisy": roll_error(noisy), "clean": roll_error(clean),
                "t_fast": clean_fast["t"], "clean_fast": roll_error(clean_fast),
                "rate": np.linalg.norm(noisy["rate_true"], axis=1)}
    return _cached("pipeline_errors", compute)


@lru_cache
def pipeline_log(controller: str = "pid") -> dict:
    """Selected arrays of the standard 7 s run."""
    def compute():
        from drone_sim.simulation.pipeline import run_pipeline

        log = run_pipeline(controller_type=controller)
        keys = ["t", "pos", "pos_des", "att_true", "att_est", "ekf_pos", "gyro_meas",
                "accel_meas", "thrust", "torques", "motor_speeds"]
        return {key: np.asarray(log[key]) for key in keys}
    return _cached(f"pipeline_{controller}", compute)


@lru_cache
def tilt_correction(duration_s: float = 30.0, rate_hz: float = 100.0) -> dict:
    """Roll estimate (deg) from a biased gyro: integration only vs Madgwick."""
    def compute():
        dt = 1.0 / rate_hz
        imu = MPU6050Simulator(MPU6050Config(sample_rate_hz=rate_hz, seed=3,
                                             gyro_constant_bias_rad_s=np.array([0.01, -0.008, 0.0])))
        t, gyro, accel = imu.simulate_static(duration_s)
        only, fused = MadgwickMARG(beta=0.0, zeta=0.0), MadgwickMARG(beta=0.05, zeta=0.0)
        roll_only, roll_fused = [], []
        for g, a in zip(gyro, accel):
            roll_only.append(only.quaternion_to_euler(only.update(g, a, dt))[0])
            roll_fused.append(fused.quaternion_to_euler(fused.update(g, a, dt))[0])
        return {"t": t, "only": np.degrees(roll_only), "fused": np.degrees(roll_fused)}
    return _cached(f"tilt_correction_{int(duration_s)}", compute)


@lru_cache
def lqr_recovery(duration_s: float = 20.0, dt: float = 0.01) -> dict:
    """The LQR recovery demo's state history (same start as ``run_recovery_demo``)."""
    def compute():
        from drone_sim.control.lqr import LQRController
        from drone_sim.dynamics.plant import DroneConfig, DronePlant

        config = DroneConfig(1.2, np.diag([0.02, 0.02, 0.04]), 0.25, 0.01, 3e-5, 1e-6)
        controller = LQRController(config)
        state = np.zeros(12)
        state[0:3] = [5.0, -0.5, 2.0]
        state[6:9] = [0.3, -0.2, 0.5]
        state[9:12] = [0.1, -0.1, 0.05]
        plant = DronePlant(config, state.copy())
        steps = int(duration_s / dt)
        history = np.zeros((steps + 1, 12))
        history[0] = state
        for k in range(steps):
            command = controller.control(state)
            thrust, torques, _ = plant.allocate_control(command[0], command[1:4])
            state = plant.step(thrust, torques, dt)
            history[k + 1] = state
        hamiltonian_eigs = np.linalg.eigvals(np.block([
            [controller.a_matrix, -controller.b_matrix @ np.linalg.inv(controller.r_matrix) @ controller.b_matrix.T],
            [-controller.q_matrix, -controller.a_matrix.T]]))
        return {"t": np.arange(steps + 1) * dt, "history": history,
                "closed_loop": controller.closed_loop_eigenvalues(), "hamiltonian": hamiltonian_eigs}
    return _cached(f"lqr_recovery_{int(duration_s)}", compute)


@lru_cache
def first_command() -> dict:
    """Requested vs allocated thrust/torque on the first PID step of the demo."""
    from drone_sim.control.pid import CascadedController
    from drone_sim.dynamics.plant import DroneConfig, DronePlant

    config = DroneConfig(1.2, np.diag([0.02, 0.02, 0.04]), 0.25, 0.01, 3e-5, 1e-6)
    state = np.zeros(12)
    state[:3] = [1.0, -0.5, -2.0]
    state[6:9] = [0.3, -0.2, 0.5]
    state[9:12] = [0.1, -0.1, 0.05]
    plant = DronePlant(config, state.copy())
    controller = CascadedController(config, 50, 500)
    thrust, torques = controller.update([1.0, -0.5, 2.0], 0.0, plant, 0.002)
    applied_thrust, applied_torques, speeds = plant.allocate_control(thrust, torques)
    return {"thrust": thrust, "torques": np.asarray(torques), "applied_thrust": applied_thrust,
            "applied_torques": applied_torques, "speeds": speeds}


@lru_cache
def gauss_markov_example(n: int = 6000, dt: float = 0.01, tau: float = 5.0, std: float = 1.0) -> dict:
    """One Gauss-Markov sequence for illustration (unit std)."""
    from drone_sim.sensors.imu_simulator import gauss_markov_bias

    rng = np.random.default_rng(7)
    return {"t": np.arange(n) * dt, "b": gauss_markov_bias(n, dt, std, tau, rng)}
