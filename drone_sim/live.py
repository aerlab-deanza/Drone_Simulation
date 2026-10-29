"""Command-line launcher for the live OpenCR IMU viewer.

Usage::

    python -m drone_sim.live --port COM5 [--baud 115200] [--rate 100]
"""

from __future__ import annotations

import argparse

from .sensors.opencr_serial import OpenCRSerialReader
from .visualization.live_viewer import LiveOpenCRViewer


def main() -> None:
    """Parse arguments, open the serial port, and run the viewer until closed."""
    parser = argparse.ArgumentParser(
        description=("Display live OpenCR attitude and virtual forward/hover motion. "
                     "IMU-only EKF position is separate and can drift.")
    )
    parser.add_argument("--port", required=True, help="serial port, e.g. COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--rate", type=float, default=100.0,
                        help="firmware sample rate in Hz")
    args = parser.parse_args()

    try:
        reader = OpenCRSerialReader(args.port, args.baud)
    except Exception as error:
        parser.error(f"could not open {args.port}: {error}")

    print(f"Reading OpenCR samples on {args.port} at {args.baud} baud.")
    print("Keep the board still until startup gyro and accelerometer calibration completes.")
    print("Displayed position is virtual 3D flight at 1.2 m/s; nose-up flies up. XY wraps at arena edge.")
    print("Close the plot window or press Ctrl+C to stop.")
    viewer = LiveOpenCRViewer(reader, sample_rate_hz=args.rate)
    try:
        viewer.show()
    except KeyboardInterrupt:
        pass
    finally:
        reader.close()


if __name__ == "__main__":
    main()
