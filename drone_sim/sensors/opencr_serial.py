"""USB-serial reader for the OpenCR IMU stream firmware.

The firmware (``firmware/opencr_imu_stream``) prints one CSV line per sample::

    timestamp_us,gx,gy,gz,ax,ay,az,mx,my,mz

* ``timestamp_us`` - board ``micros()`` timestamp.
* ``gx..gz`` - gyro rates (deg/s).
* ``ax..az`` - accelerometer specific force (g).
* ``mx..mz`` - raw magnetometer counts. All zeros means no magnetometer.

Lines starting with ``#`` are comments. Readings are converted to SI units.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: Standard gravity used to convert g to m/s².
STANDARD_GRAVITY_MPS2 = 9.80665


@dataclass(frozen=True)
class IMUSample:
    """One parsed IMU sample in SI units."""

    #: Board timestamp (µs). Wraps at 2³².
    timestamp_us: int
    #: Body rates (rad/s), board axes.
    gyro_rad_s: np.ndarray
    #: Specific force (m/s²), board axes.
    accel_m_s2: np.ndarray
    #: Raw magnetometer counts, or None if absent.
    mag_raw: np.ndarray | None


def parse_imu_line(line: str | bytes) -> IMUSample | None:
    """Parse one firmware CSV line.

    Args:
        line: Text or bytes line.

    Returns:
        The parsed sample, or None for comments, blank lines, and malformed or
        non-finite data.

    Example:
        >>> sample = parse_imu_line("1000,180,0,0,0,0,1,0,0,0")
        >>> round(float(sample.gyro_rad_s[0]), 5), float(sample.accel_m_s2[2])
        (3.14159, 9.80665)
        >>> sample.mag_raw is None
        True
    """
    if isinstance(line, bytes):
        line = line.decode("ascii", errors="ignore")
    line = line.strip()
    if not line or line.startswith("#"):
        return None

    fields = line.split(",")
    if len(fields) != 10:
        return None
    try:
        timestamp_us = int(fields[0])
        values = np.asarray([float(value) for value in fields[1:]], dtype=float)
    except ValueError:
        return None
    if not np.all(np.isfinite(values)):
        return None

    gyro_dps, accel_g, mag_raw = values[0:3], values[3:6], values[6:9]
    return IMUSample(
        timestamp_us=timestamp_us,
        gyro_rad_s=np.radians(gyro_dps),
        accel_m_s2=accel_g * STANDARD_GRAVITY_MPS2,
        mag_raw=mag_raw if np.linalg.norm(mag_raw) > 1e-9 else None,
    )


class OpenCRSerialReader:
    """Non-blocking reader of parsed samples from a USB serial port.

    Args:
        port: Serial port name, e.g. ``"COM5"`` or ``"/dev/ttyACM0"``.
        baudrate: Serial baud rate.

    Attributes:
        bad_lines: Count of non-comment lines that failed to parse.
    """

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        import serial  # pyserial; imported lazily so docs and tests need no port

        # timeout=0: the viewer polls from its animation callback, and a
        # blocking read would stall every redraw.
        self.connection = serial.Serial(port=port, baudrate=baudrate, timeout=0)
        self.connection.reset_input_buffer()
        self.bad_lines = 0
        self._pending = bytearray()

    def read_available(self, max_lines: int = 500) -> list[IMUSample]:
        """Return every complete sample received since the last call.

        Partial lines are kept until their newline arrives.

        Args:
            max_lines: Upper bound on lines parsed per call.

        Returns:
            Parsed samples, oldest first.
        """
        waiting = self.connection.in_waiting
        if waiting:
            self._pending.extend(self.connection.read(waiting))

        samples = []
        consumed = 0
        for _ in range(max_lines):
            newline = self._pending.find(b"\n", consumed)
            if newline < 0:
                break
            line = bytes(self._pending[consumed:newline + 1])
            consumed = newline + 1
            sample = parse_imu_line(line)
            if sample is None:
                if line.strip() and not line.lstrip().startswith(b"#"):
                    self.bad_lines += 1
                continue
            samples.append(sample)
        if consumed:
            del self._pending[:consumed]
        return samples

    def close(self) -> None:
        """Close the serial port if it is open."""
        if self.connection.is_open:
            self.connection.close()
