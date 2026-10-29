import unittest

import numpy as np

from drone_sim.dynamics.plant import (
    GRAVITY_MPS2 as g, DroneConfig, DronePlant, body_to_world_rotation, world_to_body_rotation,
)
from drone_sim.control.pid import AttitudeController, CascadedController, PID, PositionController
from drone_sim.control.lqr import nonlinear_dynamics


def config():
    return DroneConfig(1.0, np.diag([0.02, 0.02, 0.04]), 0.25,
                       0.0, 3e-5, 1e-6, max_motor_speed_rad_s=800.0)


class DynamicsControlTests(unittest.TestCase):
    def test_rk4_integrates_angular_acceleration_and_rate(self):
        plant = DronePlant(config(), np.zeros(12))
        dt = 0.02
        plant.step(0.0, np.array([0.0, 0.0, 0.04]), dt)
        alpha = 0.04 / plant.config.inertia[2, 2]
        self.assertAlmostEqual(plant.angular_velocity[2], alpha * dt, places=10)
        self.assertAlmostEqual(plant.euler_angles[2], 0.5 * alpha * dt**2, places=10)

    def test_world_body_rotation_are_inverses(self):
        angles = (0.3, -0.4, 0.7)
        self.assertTrue(np.allclose(world_to_body_rotation(*angles) @ body_to_world_rotation(*angles), np.eye(3)))

    def test_position_acceleration_is_yaw_aware_and_gravity_scaled(self):
        controller = PositionController(config())
        plant = DronePlant(config(), np.zeros(12))
        plant.euler_angles[2] = np.pi / 2
        controller.pid_x.kp, controller.pid_x.ki, controller.pid_x.kd = 2.0, 0.0, 0.0
        controller.pid_y.kp, controller.pid_y.ki, controller.pid_y.kd = 2.0, 0.0, 0.0
        att, thrust = controller.update([1.0, 0.0, 0.0], 0.0, plant, 0.02)
        # World +x rotates to body -y at yaw=90deg: positive roll tilts toward +y,
        # therefore the commanded roll must be negative. The angle is accel/g.
        self.assertLess(att[0], 0.0)
        self.assertAlmostEqual(att[0], -np.arctan2(2.0, g), places=6)
        self.assertAlmostEqual(thrust, plant.config.mass * g, places=8)

        # At zero yaw, positive world-y acceleration requires positive roll;
        # positive world-x acceleration requires negative pitch.
        plant.euler_angles[2] = 0.0
        controller = PositionController(config())
        att, _ = controller.update([1.0, 1.0, 0.0], 0.0, plant, 0.02)
        self.assertLess(att[1], 0.0)
        self.assertGreater(att[0], 0.0)

    def test_yaw_error_uses_shortest_path(self):
        pid = PID(1, 0, 0, np.pi - 0.01, wrap_angle_error=True)
        self.assertAlmostEqual(pid.update(-np.pi + 0.01, 0.01), -0.02, places=6)
        controller = AttitudeController(config())
        plant = DronePlant(config(), np.zeros(12))
        desired = np.array([0.0, 0.0, np.pi - 0.01])
        plant.euler_angles[2] = -np.pi + 0.01
        self.assertLess(controller.update(desired, plant, 0.01)[2], 0.0)

    def test_integral_is_bounded_and_first_derivative_is_zero(self):
        pid = PID(0.0, 1.0, 2.0, 10.0, integral_limit=0.5)
        self.assertAlmostEqual(pid.update(0.0, 0.1), 0.5)
        for _ in range(100):
            pid.update(0.0, 0.1)
        self.assertLessEqual(abs(pid.integral), 0.5)

    def test_motor_allocation_is_bounded_and_roundtrips(self):
        plant = DronePlant(config(), np.zeros(12))
        requested_thrust = plant.config.mass * g
        actual_thrust, actual_torques, speeds = plant.allocate_control(
            requested_thrust, np.array([100.0, -80.0, 30.0]))
        roundtrip_thrust, roundtrip_torques = plant.motor_mixing(speeds)
        self.assertTrue(np.all(speeds >= 0.0))
        self.assertTrue(np.all(speeds <= plant.config.max_motor_speed_rad_s))
        self.assertAlmostEqual(actual_thrust, requested_thrust, places=7)
        self.assertTrue(np.allclose([actual_thrust, *actual_torques],
                                    [roundtrip_thrust, *roundtrip_torques], atol=1e-10))
        self.assertLess(np.linalg.norm(actual_torques), np.linalg.norm([100.0, -80.0, 30.0]))

    def test_controller_scheduler_passes_accumulated_elapsed_time(self):
        controller = CascadedController(config(), outer_rate_hz=60, attitude_rate_hz=120)
        outer_elapsed, attitude_elapsed = [], []
        real_update = controller.position_control.update
        real_attitude_update = controller.attitude_control.update

        def observed_update(pos, yaw, plant, dt):
            outer_elapsed.append(dt)
            return real_update(pos, yaw, plant, dt)

        def observed_attitude_update(att, plant, dt):
            attitude_elapsed.append(dt)
            return real_attitude_update(att, plant, dt)

        controller.position_control.update = observed_update
        controller.attitude_control.update = observed_attitude_update
        plant = DronePlant(config(), np.zeros(12))
        for _ in range(7):
            controller.update([0, 0, 0], 0, plant, 0.006)
        self.assertEqual(len(outer_elapsed), 2)
        self.assertAlmostEqual(outer_elapsed[0], 0.018)
        self.assertAlmostEqual(outer_elapsed[1], 0.018)
        self.assertEqual(len(attitude_elapsed), 5)
        for actual, expected in zip(attitude_elapsed, [0.012, 0.006, 0.012, 0.006, 0.006]):
            self.assertAlmostEqual(actual, expected)

    def test_lqr_and_plant_share_the_same_derivatives_with_drag(self):
        cfg = config()
        cfg.drag_coefficient = 0.17
        state = np.array([0.2, -0.4, 0.3, 1.1, -0.6, 0.2,
                          0.25, -0.3, 0.7, 0.2, -0.1, 0.15])
        thrust, torques = 11.2, np.array([0.03, -0.02, 0.01])
        plant = DronePlant(cfg, state.copy())
        plant_derivative = plant.state_derivative(state, thrust, torques)
        lqr_derivative = nonlinear_dynamics(state, np.array([thrust, *torques]), cfg)
        self.assertTrue(np.allclose(plant_derivative, lqr_derivative, atol=1e-12))

    def test_seven_second_pid_figure_eight_tracks_with_bounded_error(self):
        duration, dt = 7.0, 1.0 / 500.0
        cfg = DroneConfig(1.2, np.diag([0.02, 0.02, 0.04]), 0.25,
                         0.01, 3e-5, 1e-6)
        state = np.zeros(12)
        state[:3] = [1.0, -0.5, -2.0]
        state[6:9] = [0.3, -0.2, 0.5]
        state[9:12] = [0.1, -0.1, 0.05]
        plant = DronePlant(cfg, state)
        controller = CascadedController(cfg, outer_rate_hz=50,
                                        attitude_rate_hz=1.0 / dt)
        actual, desired = [], []
        n_steps = int(duration / dt)
        for index in range(n_steps + 1):
            t = index * dt
            phase = 2 * np.pi * t / duration
            target = np.array([1.0 + 1.5*np.sin(phase),
                               -0.5 + 0.8*np.sin(2*phase),
                               2.0 + 1.5*t/duration])
            actual.append([plant.position[0], plant.position[1], -plant.position[2]])
            desired.append(target)
            if index < n_steps:
                thrust, torques = controller.update(target, 0.0, plant, dt)
                thrust, torques, _ = plant.allocate_control(thrust, torques)
                plant.step(thrust, torques, dt)
        error = np.asarray(actual) - np.asarray(desired)
        rmse = np.sqrt(np.mean(error**2, axis=0))
        self.assertLess(rmse[0], 0.18)
        self.assertLess(rmse[1], 0.20)
        self.assertLess(rmse[2], 0.03)
        self.assertTrue(np.all(np.isfinite(actual)))
        self.assertLessEqual(np.max(np.abs(controller.target_attitude[:2])),
                             controller.position_control.MAX_TILT + 1e-12)


if __name__ == "__main__":
    unittest.main()
