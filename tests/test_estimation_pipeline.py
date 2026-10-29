import unittest

import numpy as np

from drone_sim.dynamics.frames import (
    ned_frd_quaternion_to_up_flu,
    ned_frd_vector_to_up_flu,
    up_flu_quaternion_to_ned_frd,
)
from drone_sim.dynamics.plant import (
    body_to_world_rotation, euler_to_quaternion, GRAVITY_MPS2 as PLANT_GRAVITY_MPS2,
)
from drone_sim.estimation.ekf import IMUEKF, rotate_vector
from drone_sim.estimation.madgwick import MadgwickMARG
from drone_sim.simulation.pipeline import run_pipeline
from drone_sim.simulation.pipeline import plot_results
import matplotlib.pyplot as plt


def q_array(q):
    return np.array([q.w, q.x, q.y, q.z])


class FrameContractTests(unittest.TestCase):
    def test_rotation_and_vector_equivalence_under_frame_change(self):
        angles = np.array([0.31, -0.22, 0.47])
        rotation_ned = body_to_world_rotation(*angles)
        vector_frd = np.array([1.2, -0.7, 3.4])
        vector_up_flu = ned_frd_vector_to_up_flu(vector_frd)
        q_filter = ned_frd_quaternion_to_up_flu(q_array(euler_to_quaternion(*angles)))
        expected = ned_frd_vector_to_up_flu(rotation_ned @ vector_frd)
        np.testing.assert_allclose(rotate_vector(q_filter, vector_up_flu), expected, atol=1e-12)
        np.testing.assert_allclose(
            up_flu_quaternion_to_ned_frd(q_filter),
            q_array(euler_to_quaternion(*angles)), atol=1e-12)

    def test_initial_tilt_preserves_position_and_explicit_heading(self):
        configured_position = np.array([2.0, -3.0, 4.0])
        ekf = IMUEKF(initial_position=configured_position)
        roll, pitch, yaw = 0.25, -0.18, 0.73
        q = q_array(euler_to_quaternion(roll, pitch, yaw))
        measured_up_body = rotate_vector(q * np.array([1., -1., -1., -1.]),
                                   np.array([0., 0., 1.]))
        ekf.initialize_tilt(measured_up_body, heading=yaw)
        np.testing.assert_array_equal(ekf.position, configured_position)
        np.testing.assert_allclose(ekf.attitude, [roll, pitch, yaw], atol=1e-8)


class FilterUpdateTests(unittest.TestCase):
    def test_stationary_gravity_correction_reduces_tilt_error(self):
        truth = q_array(euler_to_quaternion(0.22, -0.16, 0.0))
        accel = rotate_vector(truth * np.array([1., -1., -1., -1.]),
                        np.array([0., 0., 9.80665]))
        madgwick = MadgwickMARG(beta=0.08, zeta=0.0)
        ekf = IMUEKF(estimate_biases=False)
        m0 = np.linalg.norm(madgwick.quaternion_to_euler(madgwick.q)[:2] - [0.22, -0.16])
        e0 = np.linalg.norm(ekf.attitude[:2] - [0.22, -0.16])
        for _ in range(700):
            madgwick.update(np.zeros(3), accel, 0.01)
            ekf.update(np.zeros(3), accel, 0.01)
        self.assertLess(np.linalg.norm(madgwick.quaternion_to_euler(madgwick.q)[:2] - [0.22, -0.16]), m0)
        self.assertLess(np.linalg.norm(ekf.attitude[:2] - [0.22, -0.16]), e0)

    def test_gravity_valid_false_prevents_translation_tilt_correction(self):
        accel = np.array([2.0, 0.0, 9.6])
        ekf = IMUEKF(estimate_biases=False)
        madgwick = MadgwickMARG(beta=0.2, zeta=0.0)
        for _ in range(100):
            ekf.update(np.zeros(3), accel, 0.01, gravity_valid=False)
            madgwick.update(np.zeros(3), accel, 0.01, gravity_valid=False)
        np.testing.assert_allclose(ekf.attitude, 0.0, atol=1e-10)
        np.testing.assert_allclose(madgwick.quaternion_to_euler(madgwick.q), 0.0, atol=1e-10)

    def test_sample_accelerometer_noise_covariance_scales_with_dt_squared(self):
        def variance(dt):
            ekf = IMUEKF(accel_noise_std=0.4, gyro_noise_std=0.0,
                         estimate_biases=False)
            initial = ekf.covariance[3, 3]
            ekf.update(np.zeros(3), np.array([0., 0., 9.80665]), dt,
                       gravity_valid=False)
            return ekf.covariance[3, 3] - initial
        short, long = variance(0.005), variance(0.01)
        self.assertAlmostEqual(long / short, 4.0, delta=0.15)

    def test_hover_specific_force_matches_shared_plant_gravity(self):
        ekf = IMUEKF(gravity=PLANT_GRAVITY_MPS2, estimate_biases=False)
        hover_specific_force = np.array([0.0, 0.0, PLANT_GRAVITY_MPS2])
        for _ in range(20):
            ekf.update(np.zeros(3), hover_specific_force, 0.01,
                       gravity_valid=False)
        np.testing.assert_allclose(ekf.velocity, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(ekf.position, np.zeros(3), atol=1e-12)

    def test_known_yaw_motion_integrates_with_gravity_observation_disabled(self):
        dt, rate, steps = 0.002, 0.4, 500
        ekf = IMUEKF(estimate_biases=False)
        madgwick = MadgwickMARG(zeta=0.0)
        gyro = np.array([0.0, 0.0, rate])
        accel = np.array([0.0, 0.0, 9.80665])
        for _ in range(steps):
            ekf.update(gyro, accel, dt, gravity_valid=False)
            madgwick.update(gyro, accel, dt, gravity_valid=False)
        expected_yaw = rate * dt * steps
        self.assertAlmostEqual(ekf.attitude[2], expected_yaw, delta=2e-3)
        self.assertAlmostEqual(madgwick.quaternion_to_euler(madgwick.q)[2], expected_yaw, delta=2e-3)


class PipelineAlignmentTests(unittest.TestCase):
    def test_pipeline_has_aligned_initial_and_final_state_sensor_logs(self):
        log = run_pipeline(duration_s=0.1, dt=0.01, seed=3)
        self.assertEqual(len(log["t"]), 11)
        np.testing.assert_allclose(log["sensor_t"], log["t"])
        np.testing.assert_allclose(log["control_t"], log["t"][:-1])
        self.assertEqual(log["pos"].shape, (11, 3))
        self.assertEqual(log["gyro_meas"].shape, (11, 3))
        np.testing.assert_allclose(log["pos"][0], [1.0, -0.5, 2.0])
        np.testing.assert_allclose(log["ekf_pos"][0], log["pos"][0], atol=1e-12)
        np.testing.assert_allclose(log["att_est"][0], log["att_true"][0], atol=1e-12)
        self.assertEqual(log["metadata"]["gravity_observation_valid"], False)
        self.assertEqual(log["metadata"]["gravity_mps2"], PLANT_GRAVITY_MPS2)
        self.assertIn("known simulation initial pose", log["metadata"]["initial_pose_reference"])

    def test_pipeline_rejects_invalid_controller_and_duration(self):
        with self.assertRaises(ValueError): run_pipeline(duration_s=0)
        with self.assertRaises(ValueError): run_pipeline(duration_s=0.105, dt=0.01)
        with self.assertRaises(ValueError): run_pipeline(controller_type="other")

    def test_plot_results_handles_n_plus_one_state_and_sensor_logs(self):
        log = run_pipeline(duration_s=0.1, dt=0.01, seed=3)
        figures = plot_results(log)
        try:
            self.assertEqual(len(figures), 6)
            self.assertEqual(len(figures[1].axes[0].lines[0].get_xdata()), len(log["t"]))
            self.assertEqual(len(figures[2].axes[0].lines[0].get_xdata()), len(log["t"]))
            self.assertEqual(len(figures[3].axes[0].lines[0].get_xdata()), len(log["t"]))
            self.assertEqual(len(figures[4].axes[0].lines[0].get_xdata()), len(log["t"]))
            self.assertEqual(len(figures[5].axes[0].lines[0].get_xdata()), len(log["sensor_t"]))
            self.assertEqual(
                figures[2].axes[0].lines[0].get_label(),
                "estimate from known initial pose",
            )
        finally:
            plt.close("all")


if __name__ == "__main__":
    unittest.main()
