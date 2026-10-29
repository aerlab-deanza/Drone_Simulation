# Other demos

## LQR recovery

```powershell
python -m drone_sim.control.lqr
```

Starts 5 m off position, tilted 0.3/−0.2/0.5 rad and spinning, and recovers to
hover at the origin with the LQR controller. It prints the closed-loop
eigenvalues (all must have negative real parts) and saves
`outputs/lqr_response.png`.

```{raw} html
<video class="demo-video" controls muted playsinline preload="metadata"
       src="../_static/videos/lqr_command.mp4"></video>
```

```{figure} /_static/figures/lqr_recovery.png
:width: 620px

LQR recovery. Every state settles within about 5 s, but only after an
aggressive transient: pitch briefly reaches about 1 rad (58°). That is far
from the hover point the controller was linearized about, and it still
converges.
```

## Waypoint mission

```powershell
python -m drone_sim.simulation.waypoint_demo
```

A 20 s mission with the cascaded PID at 5 kHz: hover, fly to (3, 2, 2.2) m
while yawing 45°, hold, return. Each leg follows a minimum-jerk profile
({func}`drone_sim.simulation.waypoint_demo.minimum_jerk`). Saves
`outputs/trajectory_3d.png`.

## Open-loop flight and gimbal lock

```powershell
python -m drone_sim.visualization.open_loop
```

Plays a scripted thrust/torque sequence with no controller, and writes the
state plots, the 3D path, and the gimbal-lock figure to
`outputs/visualizer_image.pdf`.

```{figure} /_static/figures/gimbal_lock.png
:width: 620px

Gimbal lock near 90° pitch (see {doc}`/theory/euler-angles`).
```

## IMU noise at rest

```powershell
python -m drone_sim.sensors.imu_simulator
```

10 s of a motionless MPU6050, saved to `outputs/mpu6050_simulated_data.png`.

## Central limit theorem

```powershell
python -m drone_sim.visualization.statistics_demo
```

Shows why sensor noise is modelled as Gaussian: means of uniform, exponential
and Laplace samples all converge to a normal distribution.
