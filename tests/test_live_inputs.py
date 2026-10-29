import os
import unittest

os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib.pyplot as plt
import numpy as np

from drone_sim.sensors.opencr_serial import IMUSample, parse_imu_line
from drone_sim.visualization.live_viewer import (
    LiveOpenCRViewer,
    StartupCalibration,
    horizontal_heading,
)


class FakeReader:
    def __init__(self):
        self.samples = []

    def read_available(self):
        samples, self.samples = self.samples, []
        return samples


def sample(timestamp, gyro=(0.0, 0.0, 0.0), accel=(0.0, 0.0, 9.80665), mag=None):
    return IMUSample(
        timestamp_us=timestamp,
        gyro_rad_s=np.asarray(gyro, dtype=float),
        accel_m_s2=np.asarray(accel, dtype=float),
        mag_raw=None if mag is None else np.asarray(mag, dtype=float),
    )


class LiveInputTests(unittest.TestCase):
    def test_parser_keeps_ten_field_contract_and_units(self):
        self.assertIsNone(parse_imu_line("# timestamp,gyro,accel,mag"))
        parsed = parse_imu_line("1000,180,0,-90,0,0,1,10,20,30")
        self.assertEqual(parsed.timestamp_us, 1000)
        np.testing.assert_allclose(parsed.gyro_rad_s, [np.pi, 0, -np.pi / 2])
        np.testing.assert_allclose(parsed.accel_m_s2, [0, 0, 9.80665])
        np.testing.assert_allclose(parsed.mag_raw, [10, 20, 30])
        self.assertIsNone(parse_imu_line("1,2,3"))

    def test_startup_scale_recognizes_new_and_legacy_norms(self):
        for raw_norm, expected_scale in ((9.80665, 1.0), (9.80665 / 4, 4.0), (9.80665 * 4, 0.25)):
            calibrator = StartupCalibration(3)
            accepted = None
            for index in range(3):
                accepted = calibrator.add(sample(index * 10_000, accel=(0, 0, raw_norm)))
            self.assertIsNotNone(accepted)
            self.assertEqual(accepted[0], expected_scale)
            np.testing.assert_allclose(accepted[2] * accepted[0], [0, 0, 9.80665])

    def test_fake_reader_applies_each_startup_acceleration_scale_once(self):
        for raw_norm, expected_scale in ((9.80665, 1.0), (9.80665 / 4, 4.0), (9.80665 * 4, 0.25)):
            reader = FakeReader()
            viewer = LiveOpenCRViewer(reader, sample_rate_hz=100)
            try:
                viewer.gyro_bias_sample_target = 5
                viewer.calibration = StartupCalibration(5)
                reader.samples = [sample(i * 10_000, accel=(0, 0, raw_norm)) for i in range(5)]
                viewer._update(0)
                self.assertTrue(viewer.accel_calibrated)
                self.assertEqual(viewer.accel_scale, expected_scale)
                np.testing.assert_allclose(viewer.last_sample.accel_m_s2 * viewer.accel_scale,
                                           [0, 0, 9.80665])
            finally:
                plt.close(viewer.fig)

    def test_startup_rejects_turn_then_calibrates_still_samples_without_biasing_turn(self):
        reader = FakeReader()
        viewer = LiveOpenCRViewer(reader, sample_rate_hz=100)
        try:
            viewer.gyro_bias_sample_target = 5
            viewer.calibration = StartupCalibration(5)
            reader.samples = [sample(i * 10_000, gyro=(0, 0, 0.2)) for i in range(5)]
            viewer._update(0)
            self.assertIsNone(viewer.gyro_bias_rad_s)
            self.assertIn("motion", viewer.calibration.reason)
            reader.samples = [sample((i + 5) * 10_000) for i in range(5)]
            viewer._update(1)
            self.assertIsNotNone(viewer.gyro_bias_rad_s)
            self.assertAlmostEqual(viewer.gyro_bias_rad_s[2], 0.0, places=6)
        finally:
            plt.close(viewer.fig)

    def test_unknown_scale_is_visible_and_does_not_calibrate(self):
        calibration = StartupCalibration(3)
        result = None
        for index in range(4):
            result = calibration.add(sample(index * 10_000, accel=(0, 0, 20.0)))
        self.assertIsNone(result)
        self.assertIn("invalid accelerometer scale", calibration.reason)

    def test_slow_yaw_after_calibration_is_not_erased_by_deadband(self):
        reader = FakeReader()
        viewer = LiveOpenCRViewer(reader, sample_rate_hz=100)
        try:
            viewer.gyro_bias_sample_target = 5
            viewer.calibration = StartupCalibration(5)
            reader.samples = [sample(i * 10_000) for i in range(5)]
            viewer._update(0)
            start_yaw = viewer.filter.attitude[2]
            reader.samples = [sample((i + 5) * 10_000, gyro=(0, 0, np.deg2rad(0.4))) for i in range(30)]
            viewer._update(1)
            end_yaw = viewer.filter.attitude[2]
            self.assertGreater(end_yaw - start_yaw, np.deg2rad(0.08))
        finally:
            plt.close(viewer.fig)

    def test_near_vertical_head_projection_holds_previous_heading(self):
        previous = np.array([0.0, -1.0])
        np.testing.assert_array_equal(horizontal_heading([0.1, 0.0, 0.995], previous), previous)
        np.testing.assert_allclose(horizontal_heading([0.8, 0.6, 0.0], previous), [0.8, 0.6])

    def test_stationary_magnetometer_requires_stable_field_norm(self):
        viewer = LiveOpenCRViewer(FakeReader(), sample_rate_hz=100)
        try:
            viewer.calibrated_mag_norm = 40.0
            np.testing.assert_allclose(viewer._valid_magnetometer([40, 0, 0], True), [40, 0, 0])
            self.assertIsNone(viewer._valid_magnetometer([60, 0, 0], True))
            self.assertIsNone(viewer._valid_magnetometer([40, 0, 0], False))
        finally:
            plt.close(viewer.fig)


if __name__ == "__main__":
    unittest.main()
