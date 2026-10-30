# Welcome to Drone Simulation!

```{raw} html
<div class="aerl-banner">
  <video autoplay muted loop playsinline preload="auto"
         poster="_static/figures/hero_poster.png"
         aria-label="The simulated drone flying a climbing figure-eight">
    <source src="_static/videos/hero_loop.mp4" type="video/mp4">
  </video>
</div>
```

**Drone Simulation** is a quadcopter simulator built from first principles by
the Applied Engineering Research Lab (AERL). It models the drone as a rigid
body, integrates it with fourth-order Runge-Kutta, flies it with a cascaded
PID or an LQR controller, and simulates the noisy MPU6050 IMU a real drone
would carry. Two estimators, a Madgwick filter and an extended Kalman filter,
then reconstruct the attitude from that noisy data. The same estimator also
runs **live** on the IMU of an OpenCR board.

The project is designed for learning and research rather than as a black box.
Its core objectives are:

- **First principles:** every model is derived on the {doc}`theory/index`
  pages, from coordinate frames to the Riccati equation, and the code follows
  the derivations line by line.
- **Transparency:** units and coordinate frames are stated in every docstring,
  and the {doc}`findings/index` page records what went wrong during
  development and how it was caught.
- **Reproducibility:** every figure and video in this documentation is
  generated from the package itself, and the test suite checks the docstring
  examples too.
- **Hardware in the loop:** the attitude estimator that runs in simulation also
  runs live on a real board's IMU.

Key features include a 12-state nonlinear plant with saturation-aware motor
allocation, a multi-rate cascaded PID controller and an LQR whose Riccati
equation is solved from scratch, a measured MPU6050 noise model (white noise
plus Gauss-Markov bias), and quaternion-based estimators that never hit
gimbal lock.

:::{note}
**New here?** Start with {doc}`getting-started/installation` and the
{doc}`getting-started/quickstart`. One command, `python -m drone_sim`, flies
the full demo.
:::

The simulator ships with ready-to-run scenarios, covered on the
{doc}`demos/index` pages: a climbing figure-eight flight, a waypoint mission,
an LQR recovery from a tilted, spinning start, an open-loop flight, and the
live OpenCR viewer.

Everything needed for those scenarios is included:

- **Dynamics:** NED/FRD rigid-body plant, Euler-angle kinematics, RK4
  integration, X-frame motor mixing.
- **Control:** cascaded position, attitude and rate PID loops; hover-linearized
  LQR.
- **Sensors:** MPU6050 noise model from published measurements; OpenCR
  USB-serial reader and firmware.
- **Estimation:** Madgwick gradient-descent filter (IMU and MARG); 16-state
  IMU-only extended Kalman filter.
- **Visualization:** result plots, a 3D replay, and a live 3D dashboard for
  the real board.

To extend the simulator with your own airframe or controller, see the
{doc}`api/index` and the {doc}`development` guide.

:::{tip}
Prefer watching? The {doc}`videos/index` explain gimbal lock, IMU drift,
sensor fusion and flight control with animations and on-screen captions.
:::

```{image} _static/figures/pipeline_tracking.png
:alt: Position tracking of the climbing figure-eight flight
:width: 100%
```

## Authors

1. [Kaung Thet Zaw](https://github.com/RuseaDev)
2. [Phong Nguyen](https://github.com/AlexDaPiggie)

## License

Drone Simulation is released under the
[Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0). The full
text is on the {doc}`license` page and in the repository's `LICENSE` file.
Copyright 2026 Applied Engineering Research Lab (AERL).

## Citation

If you use Drone Simulation in your work, please cite it:

```bibtex
@software{aerl_drone_simulation_2026,
  author = {{Kaung Thet Zaw} and Phong Nguyen},
  title  = {Drone Simulation: A First-Principles Quadcopter Simulator},
  year   = {2026},
  url    = {https://github.com/aerlab-deanza/Drone_Simulation},
  note   = {Version 0.1.0}
}
```

## Acknowledgement

The default IMU noise parameters are measured MPU6050 values published by
Gonzalez and Dabove (2019). The attitude filter follows Madgwick, Harrison and
Vaidyanathan (2011). The live demo runs on the ROBOTIS OpenCR board and its
Arduino IMU library.

## Table of Contents

```{toctree}
:maxdepth: 1
:caption: Getting started

getting-started/installation
getting-started/quickstart
```

```{toctree}
:maxdepth: 1
:caption: Demos

demos/index
demos/static
demos/live
demos/other
```

```{toctree}
:maxdepth: 1
:caption: Theory

theory/index
theory/frames
theory/euler-angles
theory/quaternions
theory/dynamics
theory/imu-noise
theory/madgwick
theory/ekf
theory/pid
theory/lqr
theory/notation
```

```{toctree}
:maxdepth: 1
:caption: Results

findings/index
videos/index
examples/index
```

```{toctree}
:maxdepth: 1
:caption: References

api/index
hardware
development
license
```

## Indices and tables

- {ref}`genindex`
- {ref}`modindex`
- {ref}`search`
