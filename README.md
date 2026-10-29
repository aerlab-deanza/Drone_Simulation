# Drone Simulation

A quadcopter simulator built from first principles: rigid-body dynamics with
RK4, cascaded PID and LQR control, a measured MPU6050 noise model, and
Madgwick / extended-Kalman-filter attitude estimation. It runs offline and
live from an OpenCR board's IMU.

**Documentation:** <https://aerlab-deanza.github.io/Drone_Simulation/>, with
quick start, demo videos, full theory derivations, findings, and the API
reference.

## Authors

| **Kaung Thet Zaw** | **Phong Nguyen** |
|:---|:---|
| **Project Leadership, Simulation & Control** | **Estimation & Visualization** |
| Kaung Thet Zaw (Alex) was the leader of this project. Worked on the quadcopter dynamics and RK4 integration, quaternions and frame conversions, PID and LQR controllers, motor mixing, and IMU sensor simulation. | Worked on Euler angles, the Madgwick and Kalman filters, documentation, visualization, OpenCR firmware and serial integration, the simulation pipeline and trajectories, the waypoint demo, and tests. |
| GitHub: [@RuseaDev](https://github.com/RuseaDev)<br>LinkedIn: [Kaung Thet Zaw](https://www.linkedin.com/in/kaung-thet-zaw-186b761aa/)<br>Instagram: [@applesbananasndstrawberries](https://www.instagram.com/applesbananasndstrawberries/) | GitHub: [@AlexDaPiggie](https://github.com/AlexDaPiggie)<br>LinkedIn: [Phong Nguyen](https://www.linkedin.com/in/hoai-phong-nguyen-9367a4384/) |

---

## Quick start

Run in PowerShell from the project folder:

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install -e .
```

Offline demo: a 7 s climbing figure-eight, IMU simulation, both filters, and
six plots.

```powershell
python -m drone_sim
```

Live demo: flash `firmware/opencr_imu_stream/opencr_imu_stream.ino` to an
OpenCR (Arduino IDE, board **OpenCR 1.0**), close the Serial Monitor, then keep
the board still while it calibrates:

```powershell
python -m drone_sim.live --port COM5
```

Watch the live demo: <https://www.youtube.com/watch?v=wJ4cVM0f4uc>

From Python:

```python
from drone_sim.simulation.pipeline import run_pipeline

results = run_pipeline(duration_s=7.0, controller_type="pid")  # or "lqr"
```

More demos (LQR recovery, waypoint mission, open-loop flight and gimbal lock,
IMU noise) are listed in the [demos section](https://aerlab-deanza.github.io/Drone_Simulation/demos/index.html).

## What the simulation does and does not show

- The controller uses the plant's **true** state. The filters run afterwards
  and do not feed back into control.
- Both filters start from the known initial pose: a simulation reference,
  not something an IMU can observe.
- During powered flight the accelerometer measures specific force, not
  gravity, so gravity-based tilt correction is disabled and attitude comes
  from the gyro alone. See [Findings](https://aerlab-deanza.github.io/Drone_Simulation/findings/index.html).
- The IMU-only EKF position is relative and drifts without an external
  reference (GPS, optical flow, camera tracking).

## Repository layout

| Path | Contents |
|---|---|
| [`drone_sim/dynamics`](drone_sim/dynamics/) | Plant model, rotations, quaternions, frame conversions |
| [`drone_sim/control`](drone_sim/control/) | Cascaded PID, LQR |
| [`drone_sim/sensors`](drone_sim/sensors/) | MPU6050 noise model, OpenCR serial reader |
| [`drone_sim/estimation`](drone_sim/estimation/) | Madgwick filter, EKF |
| [`drone_sim/simulation`](drone_sim/simulation/) | Figure-eight pipeline, waypoint demo |
| [`drone_sim/visualization`](drone_sim/visualization/) | Live viewer and demo plots |
| [`firmware/`](firmware/) | OpenCR IMU-stream sketch |
| [`notebooks/`](notebooks/) | Exploratory notebooks |
| [`tests/`](tests/) | `pytest` suite (also runs the docstring examples) |
| [`docs/`](docs/) | Sphinx documentation source |
| [`videos/`](videos/) | Manim theory-video sources and demo-recording scripts |

## Development

```powershell
python -m pip install -e ".[test,docs]"
python -m pytest
python -m sphinx -W --keep-going -b html docs docs/_build/html
```

See the [development guide](https://aerlab-deanza.github.io/Drone_Simulation/development.html).

## License

Released under the [Apache License 2.0](LICENSE).
Copyright 2026 Applied Engineering Research Lab (AERL). See [NOTICE](NOTICE).
