# Cascaded PID control

:::{admonition} In plain words
:class: aerl-plain
A PID controller looks at the error between where something *is* and where it
*should be*, and pushes against it. It pushes in proportion to the error (P),
to how long the error has lasted (I), and to how fast the error is changing
(D). A drone can't slide sideways directly: it has to **tilt** first. So three
PID loops are stacked. The outer loop decides how to tilt, the middle loop how
fast to rotate, and the inner loop how much torque to apply. Each inner loop
runs faster than the one around it.
:::

```{localvideo} theory/v4_control.mp4
:title: Dynamics and control: RK4, PID and LQR
```

## The key idea

```text
target position ─► [Position loop] 50 Hz ─► target tilt + thrust
target tilt     ─► [Attitude loop] 500 Hz ─► target spin rates
target spin     ─► [Rate loop] every step ─► torques ─► motors
```

Each loop uses the same PID law:

$$
u = k_p\,e + k_i \int e\,dt + k_d\,\dot e, \qquad e = \text{target} - \text{measured}.
$$

The one piece of physics in the outer loop: to accelerate sideways by $a$
while holding altitude, the drone tilts by

$$
\theta = \arctan(a/g).
$$

| Symbol | Meaning |
|---|---|
| $e$ | error: target minus measured value |
| $k_p, k_i, k_d$ | proportional, integral and derivative gains |
| $u$ | the loop's output, i.e. the next loop's target |
| $a$ | desired sideways acceleration (m/s²) |
| $g$ | gravity, 9.81 m/s² |

With the project's gains, the 7 s climbing figure-eight is tracked with an RMS
error of about 15 cm horizontally and 1 cm in altitude.

In code: {class}`~drone_sim.control.pid.CascadedController` and
{class}`~drone_sim.control.pid.PID`.

## Show the derivations

::::{dropdown} Practical details of the PID law
:icon: unfold

{class}`~drone_sim.control.pid.PID` adds three practical details:

- **Anti-windup.** $\int e\,dt$ is clamped to `integral_limit`. Otherwise a
  long saturation would build up a large integral that then overshoots.
- **No derivative kick on start.** The derivative is zero on the first sample.
- **Angle wrapping.** For yaw, $e$ is wrapped into $(-\pi, \pi]$, so a target
  of $179^\circ$ from $-179^\circ$ is a $-2^\circ$ error, not $358^\circ$
  (`test_yaw_error_uses_shortest_path`).

**Why cascade?** Rotation is fast (small inertia) and translation is slow.
Nesting the loops lets each one assume the inner loop has already reached its
set-point. That holds when the inner loop is several times faster: hence
50 Hz outside, 500 Hz in the middle, and the rate loop at the full simulation
rate.
::::

::::{dropdown} Derivation: tilt from acceleration, and tilt-compensated thrust
:icon: unfold

The x/y PIDs output desired world accelerations $a_x, a_y$ (m/s²), clipped to
$\pm 4$. They are rotated into the heading frame:

$$
a_f = \cos\psi\,a_x + \sin\psi\,a_y, \qquad a_r = -\sin\psi\,a_x + \cos\psi\,a_y .
$$

**Tilt for a given acceleration.** Tilting by $\theta$ about one axis while
holding altitude requires $T\cos\theta = mg$, and leaves a horizontal force
$T\sin\theta = mg\tan\theta$. So

$$
\theta_{\text{cmd}} = -\arctan(a_f/g), \qquad \phi_{\text{cmd}} = \arctan(a_r/g).
$$

The signs follow FRD: nose-down (negative pitch) accelerates forward,
right-wing-down (positive roll) accelerates right. Both are clipped to
$\pm 20^\circ$, which keeps the Euler-angle plant far from gimbal lock
({doc}`euler-angles`).

**Tilt-compensated thrust.** The altitude PID outputs a vertical acceleration
$a_z$ (positive up). Only the fraction $\cos\phi\cos\theta$ of the thrust
points up, so

$$
T = \frac{m\,(g + a_z)}{\max(\cos\phi\cos\theta,\ 0.1)} .
$$

The floor of 0.1 avoids dividing by nearly zero if the drone is ever close to
inverted.
::::

::::{dropdown} The attitude and rate loops, and multi-rate scheduling
:icon: unfold

**Attitude loop.** Each Euler angle has a P controller ($k_p = 6$) producing an
Euler-angle *rate*. Rates about the Euler axes are not body rates, so they are
mapped with the matrix $W(\phi, \theta)$ derived in {doc}`euler-angles`:
$\boldsymbol\omega_{\text{cmd}} = W\,[\dot\phi, \dot\theta, \dot\psi]_{\text{cmd}}^\top$.

**Rate loop.** A P controller ($k_p = 1$ N·m per rad/s) turns body-rate error
into torque. {meth}`~drone_sim.dynamics.plant.DronePlant.allocate_control` then
turns thrust and torque into feasible motor speeds ({doc}`dynamics`).

**Multi-rate scheduling.** The scheduler accumulates time and runs a loop when
its period has elapsed. It passes the **actual elapsed time** since that
loop's last run, not the nominal period, so integrals and derivatives stay
correct when the simulation step does not divide the loop period evenly
(`test_controller_scheduler_passes_accumulated_elapsed_time`).

**Gains:**

| Loop | $k_p$ | $k_i$ | $k_d$ | Output |
|---|---|---|---|---|
| altitude | 8 | 2 | 5 | m/s² |
| x, y | 10 | 0 | 10 | m/s² |
| roll, pitch, yaw | 6 | 0 | 0 | rad/s (Euler rate) |
| body rates | 1 | 0 | 0 | N·m |

The test `test_seven_second_pid_figure_eight_tracks_with_bounded_error`
checks RMS errors below 0.18 m (x), 0.20 m (y) and 0.03 m (altitude).
::::

:::{admonition} Why it matters for the drone
:class: aerl-why
- The tilt limit of ±20° is what keeps the Euler-angle plant safely away from
  gimbal lock.
- The position loop only reacts to error and doesn't know where the path goes
  next, so the drone lags the moving target slightly and overshoots the y
  peaks by about 0.2 m (see {doc}`/demos/static`).
:::
