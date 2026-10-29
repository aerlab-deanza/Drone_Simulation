"""Live Matplotlib dashboard for OpenCR IMU data.

Pipeline per serial sample::

    OpenCRSerialReader ─► startup calibration (gyro bias, accel scale)
                       ─► IMUEKF (attitude) ─► 3D board model + sensor charts

**Startup calibration.** The board must stay still for about 1.5 s. Only a
consecutive, quiet window whose accelerometer norm is about 1 g is accepted.
That window gives the gyro bias and the accelerometer scale (older firmware
streamed 0.25 g or 4 g units).

**Virtual flight.** An IMU cannot observe absolute position, so the displayed
position is *not* the EKF position. The board "flies" along its own nose
direction at :attr:`LiveOpenCRViewer.FORWARD_SPEED_M_S`, so tilting the nose up
climbs. The EKF position is frozen while the board is stationary to stop
residual acceleration from integrating into drift.

Start it with ``python -m drone_sim.live --port COM5``.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from ..estimation.ekf import IMUEKF
from ..sensors.opencr_serial import IMUSample

#: Standard gravity (m/s²).
GRAVITY_M_S2 = 9.80665
#: Accelerometer scale factors accepted at startup (new firmware is 1.0).
LEGACY_ACCEL_SCALES = (4.0, 1.0, 0.25)


class SampleSource(Protocol):
    """Anything that yields new :class:`IMUSample` objects when polled."""

    def read_available(self) -> list[IMUSample]:
        """Return the samples received since the previous call."""
        ...


def startup_accel_scale(samples: Sequence[np.ndarray] | np.ndarray,
                        gravity: float = GRAVITY_M_S2) -> float | None:
    """Pick the accelerometer scale that makes a still board read 1 g.

    Args:
        samples: Accelerometer samples (m/s² as streamed), shape ``(n, 3)``.
        gravity: Expected gravity magnitude (m/s²).

    Returns:
        One of :data:`LEGACY_ACCEL_SCALES`, or None if no scale brings the
        median norm within 20 % of ``gravity`` or the norms vary too much.

    Example:
        >>> startup_accel_scale([[0.0, 0.0, 9.80665 / 4]])
        4.0
    """
    norms = np.linalg.norm(np.asarray(samples, dtype=float), axis=1)
    median = float(np.median(norms))
    candidates = [(abs(median * scale - gravity), scale) for scale in LEGACY_ACCEL_SCALES]
    error, scale = min(candidates)
    if error > 0.20 * gravity or np.std(norms * scale) > 0.15 * gravity:
        return None
    return scale


def horizontal_heading(forward: Sequence[float], previous: Sequence[float] = (1.0, 0.0),
                       minimum_projection: float = 0.15) -> np.ndarray:
    """Return the unit horizontal direction of a 3D nose vector.

    Near-vertical poses have almost no horizontal component, so their heading
    is undefined (the same singularity as gimbal lock). The previous heading
    is kept instead.

    Args:
        forward: Nose direction in world axes.
        previous: Heading to keep for near-vertical poses.
        minimum_projection: Smallest horizontal length treated as valid.

    Returns:
        Unit 2-vector ``[x, y]``.
    """
    horizontal = np.asarray(forward, dtype=float)[:2]
    norm = float(np.linalg.norm(horizontal))
    if norm < minimum_projection:
        return np.asarray(previous, dtype=float).copy()
    return horizontal / norm


class StartupCalibration:
    """Collect a still startup window and derive gyro bias and accel scale.

    Any sample with motion, or with a non-1 g accelerometer norm, restarts
    the window. :attr:`reason` holds a human-readable status for the
    dashboard.

    Args:
        target_samples: Consecutive still samples required.
        gravity: Expected gravity magnitude (m/s²).
    """

    def __init__(self, target_samples: int, gravity: float = GRAVITY_M_S2) -> None:
        self.target_samples = target_samples
        self.gravity = gravity
        self.samples = []
        self.reason = "keep the board still for calibration"

    def add(self, sample: IMUSample) -> tuple[float, np.ndarray, np.ndarray] | None:
        """Add one sample.

        Returns:
            None while calibrating. Once the window is full and still, returns
            ``(accel_scale, gyro_bias_rad_s, mean_accel_raw)``.
        """
        gyro = np.asarray(sample.gyro_rad_s, dtype=float)
        accel = np.asarray(sample.accel_m_s2, dtype=float)
        scale = startup_accel_scale([accel], self.gravity)
        stable_gyro = np.linalg.norm(gyro) < 0.06
        if scale is None:
            self.samples.clear()
            self.reason = "invalid accelerometer scale: expected about 1 g"
            return None
        if not stable_gyro:
            self.samples.clear()
            self.reason = "motion detected; hold still to calibrate"
            return None
        if self.samples:
            prev = self.samples[-1]
            # Use relaxed per-sample thresholds: real MEMS shot noise at 100 Hz
            # routinely exceeds tight limits (e.g. 0.025 rad/s gyro diff) even
            # at rest, causing the window to reset before it can fill.
            if (np.linalg.norm(accel - prev.accel_m_s2) > 0.50
                    or np.linalg.norm(gyro - prev.gyro_rad_s) > 0.08):
                self.samples.clear()
                self.reason = "motion detected; hold still to calibrate"
        self.samples.append(sample)
        if len(self.samples) < self.target_samples:
            self.reason = f"calibrating {len(self.samples)}/{self.target_samples}; hold still"
            return None
        gyro_array = np.asarray([s.gyro_rad_s for s in self.samples])
        accel_array = np.asarray([s.accel_m_s2 for s in self.samples])
        if (np.max(np.std(gyro_array, axis=0)) >= 0.05
                or np.max(np.std(accel_array, axis=0)) >= 0.15):
            self.samples.clear()
            self.reason = "startup window was moving; hold still to retry"
            return None
        scale = startup_accel_scale(accel_array, self.gravity)
        if scale is None:
            self.samples.clear()
            self.reason = "invalid accelerometer scale: expected about 1 g"
            return None
        self.reason = "calibrated"
        return scale, np.mean(gyro_array, axis=0), np.mean(accel_array, axis=0)


def _rotate_vectors(q: np.ndarray, vectors: np.ndarray) -> np.ndarray:
    """Rotate row vectors from body to world with quaternion ``[w, x, y, z]``."""
    w, x, y, z = q
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return np.asarray(vectors) @ rotation.T


class LiveOpenCRViewer:
    """Animated dashboard: 3D board model, virtual path, attitude and IMU charts.

    Args:
        reader: Sample source, normally :class:`~drone_sim.sensors.OpenCRSerialReader`.
        sample_rate_hz: Firmware sample rate (Hz), used as the fallback ``dt``
            and to size the calibration window.
        history_seconds: Length of the scrolling sensor charts (s).

    Attributes:
        filter: The attitude :class:`~drone_sim.estimation.IMUEKF` (biases
            disabled, since the startup calibration removes them).
        sim_position: Virtual position shown in 3D (m).
    """

    #: Initial virtual altitude (m).
    HOVER_HEIGHT_M = 1.5
    #: Virtual forward speed along the board's nose (m/s).
    FORWARD_SPEED_M_S = 1.20

    def __init__(self, reader: SampleSource, sample_rate_hz: float = 100.0,
                 history_seconds: float = 10.0) -> None:
        self.reader = reader
        # Live gyro offset is calibrated below; keeping the EKF's separate bias
        # states active would estimate the same offsets a second time.
        self.filter = IMUEKF(estimate_biases=False)
        self.nominal_dt = 1.0 / sample_rate_hz
        self.last_timestamp_us = None
        self.elapsed_s = 0.0
        self.history_limit = max(100, int(sample_rate_hz * history_seconds))
        # Fixed-size numeric buffers avoid converting Python deques to arrays on
        # every animation frame. Keep the 3D trail shorter than chart history.
        self.times = np.zeros(self.history_limit)
        self.gyro_history = np.zeros((self.history_limit, 3))
        self.accel_history = np.zeros((self.history_limit, 3))
        self.position_history = np.zeros((min(self.history_limit, 300), 3))
        self.history_count = 0
        self.history_next = 0
        self.path_count = 0
        self.path_next = 0
        self.smoothed_position = None
        self.position_smoothing_alpha = 0.25
        self.chart_stride = 2
        self.frame_count = 0
        self.last_sample = None
        self.samples_seen = 0
        self.gyro_bias_sample_target = max(50, int(sample_rate_hz * 1.5))
        self.calibration = StartupCalibration(self.gyro_bias_sample_target)
        self.gyro_bias_rad_s = None
        self.accel_scale = 1.0
        self.accel_calibrated = False
        self.calibrated_mag_norm = None
        self.mag_norm_history = []
        self.horizontal_heading = np.array([1.0, 0.0])
        self.recent_accel = []
        # Open scene: start on screen-right, move forward without walls, and
        # maintain a fixed hover altitude.
        self.sim_position = np.array([-1.2, 0.0, self.HOVER_HEIGHT_M])

        self.fig = plt.figure(figsize=(11.0, 6.5))
        # A side-by-side dashboard is too wide on a narrow laptop/remote
        # desktop. Detect the display before placing axes and stack panels
        # vertically when the available screen width is limited.
        manager = plt.get_current_fig_manager()
        window = getattr(manager, "window", None)
        try:
            compact_layout = window.winfo_screenwidth() < 1400
        except (AttributeError, TypeError):
            compact_layout = False
        self.compact_layout = compact_layout
        if compact_layout:
            self.fig.set_size_inches(7.2, 7.4, forward=False)
        self.fig.suptitle("Live Simulation")
        if compact_layout:
            grid = self.fig.add_gridspec(
                3, 1, height_ratios=(1.25, 1.0, 1.0), hspace=0.42
            )
            self.board_ax = self.fig.add_subplot(grid[0, 0], projection="3d")
            self.gyro_ax = self.fig.add_subplot(grid[1, 0])
            self.accel_ax = self.fig.add_subplot(grid[2, 0])
        else:
            grid = self.fig.add_gridspec(
                2, 2, width_ratios=(0.82, 1.18), wspace=0.30, hspace=0.25
            )
            self.board_ax = self.fig.add_subplot(grid[:, 0], projection="3d")
            self.gyro_ax = self.fig.add_subplot(grid[0, 1])
            self.accel_ax = self.fig.add_subplot(grid[1, 1])
        self._configure_board_axes()
        # Keep the numeric readout in figure space. Text anchored to a 3D axis
        # can be clipped or hidden by the 3D panes as their limits change.
        self.board_text = self.fig.text(
            0.045,
            0.90,
            "Waiting for OpenCR samples…\nCheck firmware, USB cable, and port.",
            va="top", family="monospace", fontsize=9,
            bbox={"boxstyle": "round,pad=0.18", "facecolor": "white", "alpha": 0.9},
        )

        # Enlarge the board model for screen visibility; this is a display
        # scale only and does not change the IMU position estimate.
        display_scale = 2.2
        vertices = display_scale * np.array([
            [-0.13, -0.09, -0.015], [0.13, -0.09, -0.015],
            [0.13, 0.09, -0.015], [-0.13, 0.09, -0.015],
            [-0.13, -0.09, 0.015], [0.13, -0.09, 0.015],
            [0.13, 0.09, 0.015], [-0.13, 0.09, 0.015],
        ])
        self.body_vertices = vertices
        self.body_faces = [
            [0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4],
            [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7],
        ]
        self.body = Poly3DCollection(
            [[self.body_vertices[i] for i in face] for face in self.body_faces],
            # Mark the +X end as the board head.
            facecolors=["#263746", "#4f9bd3", "#90a4ae", "#e53935", "#90a4ae", "#607d8b"],
            edgecolors="#102a43", linewidths=1.4, alpha=0.98,
        )
        self.board_ax.add_collection3d(self.body)
        self.axis_lines = [
            self.board_ax.plot([], [], [], color=color, linewidth=2.5,
                               marker="o", markersize=4)[0]
            for color in ("#e53935", "#43a047", "#1e88e5")
        ]
        self.head_line = self.board_ax.plot(
            [], [], [], color="#e53935", linewidth=2.0, linestyle="-"
        )[0]
        self.path_line = self.board_ax.plot([], [], [], color="#ff8f00", linewidth=2.2)[0]

        colors = ("#e53935", "#43a047", "#1e88e5")
        axis_names = ("X", "Y", "Z")
        self.gyro_lines = [
            self.gyro_ax.plot([], [], color=colors[i], label=axis_names[i])[0]
            for i in range(3)
        ]
        self.accel_lines = [
            self.accel_ax.plot([], [], color=colors[i], label=axis_names[i])[0]
            for i in range(3)
        ]
        self._configure_signal_axes()
        self.animation = FuncAnimation(
            self.fig, self._update, interval=30, blit=False, cache_frame_data=False
        )

    def _configure_board_axes(self):
        self.board_ax.set_title("")
        self.board_ax.set_xlabel("local X [m]")
        self.board_ax.set_ylabel("local Y [m]")
        self.board_ax.set_zlabel("altitude [m]")
        self.board_ax.set_proj_type("ortho")
        # View from the opposite side so +X appears on screen-left while all
        # three coordinate axes retain their normal increasing direction.
        self.board_ax.view_init(elev=25, azim=120)
        self.board_ax.tick_params(labelsize=9)
        self.board_ax.set_xlim(-2.0, 2.0)
        self.board_ax.set_ylim(-2.0, 2.0)
        self.board_ax.set_zlim(0.0, 3.0)
        self.board_ax.set_box_aspect((1.0, 1.0, 0.75))

    def _configure_signal_axes(self):
        self.gyro_ax.set_title("Gyroscope")
        self.gyro_ax.set_ylabel("rad/s")
        self.gyro_ax.grid(alpha=0.25)
        self.gyro_ax.legend(loc="upper right", ncol=3, fontsize=8)
        self.accel_ax.set_title("Accelerometer")
        self.accel_ax.set_xlabel("time since start [s]")
        self.accel_ax.set_ylabel("m/s²")
        self.accel_ax.grid(alpha=0.25)
        self.accel_ax.legend(loc="upper right", ncol=3, fontsize=8)
        self.fig.tight_layout(rect=(0.02, 0.01, 0.98, 0.95), pad=1.0)
        if self.compact_layout:
            self.fig.subplots_adjust(left=0.10, right=0.96, bottom=0.11,
                                     top=0.91, hspace=0.50)
        else:
            self.fig.subplots_adjust(
                left=0.035, right=0.96, bottom=0.14, top=0.90, wspace=0.30
            )

    def _sample_dt(self, timestamp_us: int) -> float:
        """Return seconds since the previous sample, handling ``micros()`` wrap."""
        if self.last_timestamp_us is None:
            dt = self.nominal_dt
        else:
            # Unsigned subtraction also handles micros() wrapping after ~71 min.
            delta_us = (timestamp_us - self.last_timestamp_us) & 0xFFFFFFFF
            dt = delta_us / 1_000_000.0
            if not 1e-5 <= dt <= 0.25:
                dt = self.nominal_dt
        self.last_timestamp_us = timestamp_us
        self.elapsed_s += dt
        return dt

    def _update(self, _frame: int) -> list:
        """Animation callback: consume new samples, then redraw."""
        for sample in self.reader.read_available():
            dt = self._sample_dt(sample.timestamp_us)
            gyro_model = sample.gyro_rad_s
            accel_sensor = sample.accel_m_s2 * self.accel_scale
            mag_model = sample.mag_raw
            if self.gyro_bias_rad_s is None:
                calibration = self.calibration.add(sample)
                if calibration is None:
                    self.last_sample = sample
                    self.samples_seen += 1
                    self._append_history(sample, self.sim_position, accel=accel_sensor)
                    continue
                self.accel_scale, self.gyro_bias_rad_s, initial_accel = calibration
                self.accel_calibrated = True
                # Earlier stream versions can be 0.25 g or 4 g; the new
                # firmware is already physical g and gets scale 1.0.
                self.accel_history[:self.history_count] *= self.accel_scale
                self.calibrated_mag_norm = self._startup_mag_norm(self.calibration.samples)
                initial_accel_norm = np.linalg.norm(initial_accel)
                initial_accel *= self.accel_scale
                initial_accel_norm = np.linalg.norm(initial_accel)
                if 0.85 * self.filter.gravity <= initial_accel_norm <= 1.15 * self.filter.gravity:
                    self.filter.initialize_tilt(initial_accel)

            corrected_accel = sample.accel_m_s2 * self.accel_scale
            accel_norm = np.linalg.norm(corrected_accel)
            self.recent_accel.append(corrected_accel.copy())
            self.recent_accel = self.recent_accel[-10:]
            gravity_consistent = 0.85 * self.filter.gravity <= accel_norm <= 1.15 * self.filter.gravity
            # Use only the startup gyro calibration. Continuously adapting
            # bias during slow motion can mistake deliberate pitch/yaw for drift.
            corrected_gyro = gyro_model - self.gyro_bias_rad_s
            stationary = (
                np.linalg.norm(corrected_gyro) < 0.04
                and gravity_consistent
                and len(self.recent_accel) == 10
                and np.max(np.std(self.recent_accel, axis=0)) < 0.15
            )
            valid_mag = self._valid_magnetometer(mag_model, stationary)
            position_before_update = self.filter.position
            position, _velocity, _attitude = self.filter.update(
                corrected_gyro,
                corrected_accel,
                dt,
                mag=valid_mag,
                gravity_valid=stationary and gravity_consistent,
            )
            if stationary:
                # An IMU cannot observe absolute position while resting. Prevent
                # tiny acceleration residuals from integrating into visible
                # translation and a wandering path.
                self.filter.state[IMUEKF.POSITION] = position_before_update
                self.filter.state[IMUEKF.VELOCITY] = 0.0
                position = self.filter.position
            # Move along the board's full 3D +X/head direction at a fixed
            # speed. Tilting the nose up drives the drone upward.
            forward = _rotate_vectors(self.filter.quaternion, np.array([[1.0, 0.0, 0.0]]))[0]
            fwd_norm = np.linalg.norm(forward)
            if fwd_norm > 1e-9:
                forward = forward / fwd_norm
            self.sim_position += forward * self.FORWARD_SPEED_M_S * dt
            # No hard boundaries on XY — the camera follows the drone so the
            # space feels infinite and open. Only clamp Z so the drone stays
            # within the visible altitude range.
            self.sim_position[2] = np.clip(self.sim_position[2], 0.05, 2.95)
            self.last_sample = sample
            self.samples_seen += 1
            self._append_history(sample, self.sim_position, accel=corrected_accel)

        if self.last_sample is None:
            return []

        self.frame_count += 1
        # Refresh the board and numeric readout on every animation frame;
        # refresh the heavier sensor charts every other frame.
        roll, pitch, yaw = self.filter.attitude
        # Render the EKF quaternion directly, without display-only angle flips.
        q = self.filter.quaternion
        center = self.sim_position.copy()
        rotated = _rotate_vectors(q, self.body_vertices) + center
        self.body.set_verts([[rotated[i] for i in face] for face in self.body_faces])
        axis_vectors = _rotate_vectors(q, np.eye(3)) * 0.42
        for index, line in enumerate(self.axis_lines):
            endpoint = center + axis_vectors[index]
            line.set_data_3d(
                [center[0], endpoint[0]],
                [center[1], endpoint[1]],
                [center[2], endpoint[2]],
            )
        head = center + axis_vectors[0]
        self.head_line.set_data_3d(
            [center[0], head[0]], [center[1], head[1]], [center[2], head[2]]
        )
        path = self._ordered_path()
        self.path_line.set_data_3d(path[:, 0], path[:, 1], path[:, 2])
        # Camera follows the drone every frame — no snap threshold.
        # The view window slides smoothly so the space feels open and infinite.
        radius = 2.0
        self.board_ax.set_xlim(center[0] - radius, center[0] + radius)
        self.board_ax.set_ylim(center[1] - radius, center[1] + radius)
        self.board_ax.set_zlim(0.0, 3.0)
        roll, pitch, yaw = np.degrees((roll, pitch, yaw))
        position_text = ", ".join(f"{value:+.2f}" for value in center)
        if not self.accel_calibrated:
            calibration_text = f"\n{self.calibration.reason}"
        else:
            calibration_text = "\nvirtual: 1.2 m/s 3D, open space\nZ clamped 0-3 m"
        self.board_text.set_text(
            f"roll  {roll:+7.1f}°\n"
            f"pitch {pitch:+7.1f}°\n"
            f"yaw   {yaw:+7.1f}°\n"
            "axes: roll X | pitch Y | yaw Z\n"
            f"virtual position [m]\n{position_text}{calibration_text}"
        )

        if self.frame_count % self.chart_stride == 0:
            times, gyro, accel = self._ordered_history()
            for index in range(3):
                self.gyro_lines[index].set_data(times, gyro[:, index])
                self.accel_lines[index].set_data(times, accel[:, index])
            if len(times) > 1:
                left = max(0.0, times[-1] - 10.0)
                right = max(10.0, times[-1])
                self.gyro_ax.set_xlim(left, right)
                self.accel_ax.set_xlim(left, right)
                self.gyro_ax.relim()
                self.gyro_ax.autoscale_view(scalex=False, scaley=True)
                self.accel_ax.relim()
                self.accel_ax.autoscale_view(scalex=False, scaley=True)
        return []

    def _append_history(self, sample, position, accel=None):
        index = self.history_next
        self.times[index] = self.elapsed_s
        self.gyro_history[index] = sample.gyro_rad_s
        self.accel_history[index] = (
            sample.accel_m_s2 if accel is None else accel
        )
        self.history_next = (index + 1) % self.history_limit
        self.history_count = min(self.history_count + 1, self.history_limit)

        position = np.asarray(position, dtype=float)
        if self.smoothed_position is None:
            self.smoothed_position = position.copy()
        else:
            self.smoothed_position += self.position_smoothing_alpha * (
                position - self.smoothed_position
            )
        path_index = self.path_next
        self.position_history[path_index] = self.smoothed_position
        self.path_next = (path_index + 1) % len(self.position_history)
        self.path_count = min(self.path_count + 1, len(self.position_history))

    @staticmethod
    def _startup_mag_norm(samples):
        norms = [np.linalg.norm(s.mag_raw) for s in samples if s.mag_raw is not None]
        if len(norms) < 10:
            return None
        norms = np.asarray(norms)
        median = float(np.median(norms))
        if median <= 1e-9 or np.std(norms) / median > 0.10:
            return None
        return median

    def _valid_magnetometer(self, mag: np.ndarray | None,
                            stationary: bool) -> np.ndarray | None:
        """Return ``mag`` only if it is trustworthy for a heading update.

        The board must be stationary, the field norm within 20 % of the
        startup norm, and the direction within 45° of the prediction.
        """
        if not stationary or self.calibrated_mag_norm is None or mag is None:
            return None
        mag = np.asarray(mag, dtype=float)
        norm = float(np.linalg.norm(mag))
        if norm <= 1e-9 or abs(norm / self.calibrated_mag_norm - 1.0) > 0.20:
            return None
        # Once a field reference exists, suppress large heading innovations;
        # this catches nearby magnetic disturbances without claiming calibrated
        # absolute heading accuracy.
        field_world = self.filter.magnetic_field_world
        if field_world is not None:
            predicted = self.filter.magnetic_measurement_model(self.filter.state, field_world)
            cosine = np.clip(np.dot(mag / norm, predicted / np.linalg.norm(predicted)), -1.0, 1.0)
            if np.arccos(cosine) > np.deg2rad(45.0):
                return None
        return mag

    def _ordered_history(self):
        if self.history_count < self.history_limit:
            selection = slice(0, self.history_count)
            return (self.times[selection], self.gyro_history[selection],
                    self.accel_history[selection])
        indexes = np.concatenate((np.arange(self.history_next, self.history_limit),
                                  np.arange(0, self.history_next)))
        return self.times[indexes], self.gyro_history[indexes], self.accel_history[indexes]

    def _ordered_path(self):
        if self.path_count < len(self.position_history):
            return self.position_history[:self.path_count]
        indexes = np.concatenate((np.arange(self.path_next, len(self.position_history)),
                                  np.arange(0, self.path_next)))
        return self.position_history[indexes]

    def show(self) -> None:
        """Open the window and block until it is closed."""
        plt.show()
