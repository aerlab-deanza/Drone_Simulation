"""Quadcopter simulation: dynamics, control, IMU sensing, and state estimation.

Subpackages:

* :mod:`drone_sim.dynamics` - rigid-body plant, rotations, frame conventions.
* :mod:`drone_sim.control` - cascaded PID and LQR controllers.
* :mod:`drone_sim.sensors` - MPU6050 noise model and OpenCR serial reader.
* :mod:`drone_sim.estimation` - Madgwick filter and IMU-only EKF.
* :mod:`drone_sim.simulation` - end-to-end scenarios.
* :mod:`drone_sim.visualization` - plots, demos, live dashboard.
"""

__version__ = "0.1.0"
