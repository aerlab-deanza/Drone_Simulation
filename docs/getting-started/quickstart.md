# Quick start

## Run the offline (static) demo

```powershell
python -m drone_sim
```

This flies a 7-second climbing figure-eight with the cascaded PID controller,
simulates the IMU, runs both filters, prints a summary, and opens six plot
windows:

```text
Running 7 seconds of simulation at 500 Hz...
Expected path: one figure-eight loop with a steady climb
  altitude rises from 2.0 m to 3.5 m
  it ends above its starting point
Final position (x, y, alt): [ 0.945 -0.61   3.502] m
Final expected position (x, y, alt): [ 1.  -0.5  3.5] m
EKF displacement from its known initial pose (x, y, altitude): [-0.083 -0.159  1.535] m (drifts without external position data)
Final attitude estimation error (roll, pitch, yaw): [-0.054  0.012  0.014] deg
Simulation finished. Opening plots...
```

Close the plot windows to exit. {doc}`/demos/static` explains every plot.

## Run the live demo with an OpenCR board

1. Flash `firmware/opencr_imu_stream/opencr_imu_stream.ino` to the OpenCR with
   the Arduino IDE (see {doc}`/hardware`).
2. Find the board's serial port (Device Manager → *Ports (COM & LPT)*, e.g.
   `COM5`).
3. Start the viewer and **keep the board still** for about 1.5 s while it
   calibrates:

```powershell
python -m drone_sim.live --port COM5
```

Tilt the board and the 3D model follows. {doc}`/demos/live` explains the
display.

## Use it from Python

```python
from drone_sim.simulation.pipeline import run_pipeline, plot_results

log = run_pipeline(duration_s=7.0, controller_type="lqr")
print(log["pos"][-1])          # final (x, y, altitude) in metres
figures = plot_results(log)    # six Matplotlib figures
```

`run_pipeline` returns a dictionary of NumPy arrays; its docstring
({func}`drone_sim.simulation.pipeline.run_pipeline`) lists every key with its
unit and frame.

## Other entry points

| Command | What it does |
|---|---|
| `python -m drone_sim.control.lqr` | LQR recovery from a tilted, spinning start |
| `python -m drone_sim.simulation.waypoint_demo` | 20 s waypoint mission with yaw |
| `python -m drone_sim.visualization.open_loop` | Open-loop RK4 flight + gimbal-lock plots (PDF) |
| `python -m drone_sim.sensors.imu_simulator` | 10 s of simulated MPU6050 data at rest |
| `python -m drone_sim.visualization.statistics_demo` | Central limit theorem demo |

Outputs are written to `outputs/`.
