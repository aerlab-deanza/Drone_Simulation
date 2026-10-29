# Static (offline) demo

```powershell
python -m drone_sim
```

## Running the command

A real screen recording. The command runs, the six plot windows open, and they
are ordinary interactive Matplotlib windows: drag inside the 3D plot with the
mouse to rotate it, and maximize, restore or resize any window by its title bar
or corner.

```{raw} html
<video class="demo-video" controls muted playsinline preload="metadata"
       src="../_static/videos/static_command.mp4?v=2"></video>
```

## 3D replay

A rendered replay of the same run. The drone model is oriented by the **true**
attitude; the orange ghost uses the **Madgwick estimate**, and the trails show
the true path, the reference, and the IMU-only EKF position.

```{raw} html
<video class="demo-video" controls muted playsinline preload="metadata"
       src="../_static/videos/static_replay.mp4"></video>
```

## What happens

1. **Fly.** The cascaded PID controller ({doc}`/theory/pid`) follows a climbing
   figure-eight for 7 s at 500 Hz. It starts deliberately tilted
   (roll 0.3 rad, pitch −0.2 rad, yaw 0.5 rad) and spinning, so the first half
   second shows recovery. The controller reads the plant's **true** state.
2. **Sense.** The true body rates and specific force are passed through the
   MPU6050 noise model ({doc}`/theory/imu-noise`).
3. **Estimate.** The Madgwick filter and the EKF process the noisy samples
   afterwards; they do not influence the flight.

During powered flight the accelerometer measures thrust and acceleration as
well as gravity, so its direction is not "down". Both filters are therefore
told `gravity_valid=False` and attitude comes from gyro integration alone. This
deliberately exposes raw gyro drift (see {doc}`/findings/index`).

## Reading the plots

```{figure} /_static/figures/pipeline_path_3d.png
:width: 480px

**3D path.** Blue: actual flight. Red dashed: reference. The drone starts on
the reference but tilted and spinning, so it falls behind during the first
second before catching up.
```

```{figure} /_static/figures/pipeline_tracking.png
:width: 600px

**Position tracking** per axis. Altitude tracks within centimetres. x and y
lag the moving target and overshoot the y peaks by about 0.2 m: the position
loop reacts only to error and has no feed-forward of the reference velocity.
```

```{figure} /_static/figures/pipeline_attitude.png
:width: 600px

**Attitude: truth vs Madgwick.** The curves overlap at this scale.
```

```{figure} /_static/figures/pipeline_attitude_error.png
:width: 600px

**Attitude estimation error.** Peaks of about 0.15° during the fast
recovery manoeuvre in the first 2 s, about 0.05° at the end. Surprisingly,
most of it is **not** sensor noise: a noise-free IMU gives almost the same
curve. It is the error of the filter's first-order integration step
($q + \dot q\,\Delta t$), which grows with rotation rate and shrinks 4× when
the sample rate is raised 4×. The MPU6050 noise alone would contribute only
about 0.01° in 7 s (see {doc}`/theory/imu-noise` and {doc}`/findings/index`).
```

```{figure} /_static/figures/pipeline_ekf_position.png
:width: 600px

**IMU-only EKF position.** It follows the flight closely for 7 s, but nothing
bounds its error: position is a double integral of noisy acceleration.
```

```{figure} /_static/figures/pipeline_imu.png
:width: 600px

**Simulated IMU stream** (FLU body axes). The accelerometer z axis sits near
+9.81 m/s² at hover: an accelerometer measures *specific force*, the support
force per unit mass, not gravity itself.
```

## Choosing the controller

From Python, swap PID for LQR:

```python
from drone_sim.simulation.pipeline import run_pipeline
log = run_pipeline(controller_type="lqr")
```

```{figure} /_static/figures/pipeline_path_3d_lqr.png
:width: 480px

The same figure-eight with the hover-linearized LQR. It tracks less tightly:
it swings wide of both loops. LQR is optimal for small deviations from hover,
and the reference here is a moving target it was not designed to follow.
```
