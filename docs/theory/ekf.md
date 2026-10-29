# The extended Kalman filter

:::{admonition} In plain words
:class: aerl-plain
The extended Kalman filter (EKF) keeps two things: its **best guess** of the
drone's state and **how unsure** it is about each part of that guess. Every
IMU sample moves the guess forward and makes it a little *more* unsure. When a
trustworthy measurement arrives (the direction of gravity, or north), the
filter corrects the guess and becomes *less* unsure, but only in the
directions that measurement can actually see. Nothing an IMU measures can see
**position**, so the position estimate is only relative, and it drifts.
:::

```{localvideo} theory/v3_fusion.mp4
:title: Sensor fusion: Madgwick and the EKF
```

## The key idea

The filter repeats two steps for every sample:

| Step | What happens | Uncertainty |
|---|---|---|
| **Predict** | The gyro turns the attitude; the accelerometer, with gravity removed, moves velocity and position | grows |
| **Correct** | Compare the predicted "down" (and "north") with the measurement, and move the guess toward it | shrinks, but only where the measurement can see |

How far to move is set by the **Kalman gain** $K$. It weighs how unsure the
prediction is ($P$) against how noisy the measurement is ($R_z$):

$$
K = \frac{\text{prediction uncertainty}}{\text{prediction uncertainty} + \text{measurement noise}}
\;\approx\; P\,(P + R_z)^{-1}.
$$

A confident prediction moves little; an unsure one moves a lot.

| Symbol | Meaning |
|---|---|
| $\mathbf x$ | the 16-number state: position, velocity, attitude quaternion, gyro bias, accelerometer bias |
| $P$ | the state's uncertainty (a 16×16 covariance matrix) |
| $R_z$ | the measurement's noise |
| $K$ | the Kalman gain: how much to trust the measurement |

**What an IMU can and cannot see:**

| State | Seen by | Observable? |
|---|---|---|
| roll, pitch | gravity | yes, while the drone isn't accelerating |
| yaw (heading) | magnetometer only | only with a magnetometer |
| **position, velocity** | **nothing** | **never**: the uncertainty only grows |

In code: {class}`~drone_sim.estimation.ekf.IMUEKF`.

## Show the derivations

::::{dropdown} Prediction: strapdown mechanization and the covariance
:icon: unfold

The state is

$$
\mathbf x = [\,\mathbf p\ (3),\ \mathbf v\ (3),\ q\ (4),\ \mathbf b_g\ (3),\ \mathbf b_a\ (3)\,]
\in \mathbb R^{16},
$$

in the z-up world / FLU body frames, with covariance $P \in \mathbb R^{16\times16}$.
Each IMU sample $(\tilde{\boldsymbol\omega}, \tilde{\mathbf f})$ drives the state
forward ({meth}`~drone_sim.estimation.ekf.IMUEKF.transition`):

$$
\begin{aligned}
q &\leftarrow \operatorname{normalize}\!\big(q + \tfrac12 q \otimes (0, \tilde{\boldsymbol\omega} - \mathbf b_g)\,\Delta t\big)\\
\mathbf a &= R(q)\,(\tilde{\mathbf f} - \mathbf b_a) - [0, 0, g]^\top\\
\mathbf p &\leftarrow \mathbf p + \mathbf v\,\Delta t + \tfrac12 \mathbf a\,\Delta t^2\\
\mathbf v &\leftarrow \mathbf v + \mathbf a\,\Delta t
\end{aligned}
$$

The accelerometer measures specific force, so gravity must be *subtracted*
after rotating into the world frame. Errors in $q$ therefore leak gravity
into $\mathbf a$ ({doc}`imu-noise`).

The covariance follows the linearized dynamics,
$P \leftarrow F P F^\top + Q$, with $F = \partial(\text{transition})/\partial\mathbf x$.
Most of $F$ is written analytically (e.g. $\partial\mathbf p/\partial\mathbf v = I\,\Delta t$,
$\partial\mathbf v/\partial\mathbf b_a = -R\,\Delta t$). Only the blocks involving
$q$ and $\mathbf b_g$ are differentiated numerically, which avoids 32
transition evaluations per sample.

**Process noise.** The constructor's noise values are per-sample standard
deviations $\sigma_a, \sigma_g$. They enter through the same discrete map as
the measurements: $Q = G_a \Sigma_a G_a^\top + G_g \Sigma_g G_g^\top + Q_b\Delta t$,
with $G_a = [\tfrac12 R\Delta t^2;\ R\Delta t;\ 0]$. Keeping $G_a$ intact
preserves the position–velocity correlation, and makes the velocity variance
grow as $\sigma_a^2\Delta t^2$ per step. The test
`test_sample_accelerometer_noise_covariance_scales_with_dt_squared` checks
this scaling.
::::

::::{dropdown} Correction: the Kalman gain and the Joseph form
:icon: unfold

For a measurement $\mathbf z$ with model $h(\mathbf x)$, Jacobian
$H = \partial h/\partial\mathbf x$, and noise covariance $R_z$:

$$
\begin{aligned}
S &= H P H^\top + R_z, \qquad K = P H^\top S^{-1},\\
\mathbf x &\leftarrow \mathbf x + K\,(\mathbf z - h(\mathbf x)),\\
P &\leftarrow (I - KH)\,P\,(I - KH)^\top + K R_z K^\top \quad\text{(Joseph form).}
\end{aligned}
$$

**Why the Joseph form?** The textbook update $P \leftarrow (I - KH)P$ is only
correct for the exact optimal gain, and rounding can make it lose symmetry or
positive definiteness. The Joseph form is a sum of two matrices of the form
$A P A^\top$ and $K R_z K^\top$, both symmetric positive semidefinite, so the
result stays a valid covariance even with a slightly wrong $K$.

Two measurements are available:

Gravity
: $h_g(\mathbf x) = R(q)^\top [0, 0, g]^\top + \mathbf b_a$
  ({meth}`~drone_sim.estimation.ekf.IMUEKF.gravity_measurement_model`). Used
  only if the caller sets `gravity_valid=True` **and**
  $0.75g \le \|\tilde{\mathbf f}\| \le 1.25g$. The norm gate rejects shocks,
  but it cannot detect a steady acceleration, hence the caller's flag.

Magnetometer
: $h_m(\mathbf x) = R(q)^\top \hat{\mathbf m}_{\text{world}}$, a unit direction
  ({meth}`~drone_sim.estimation.ekf.IMUEKF.magnetic_measurement_model`). If no
  reference is given, the first sample defines "north".

**Rate-adaptive trust.** During fast turns, vibration and centripetal
acceleration corrupt both reference vectors. Their noise is inflated by
$1 + (\|\boldsymbol\omega\|/0.45)^2$, so at 0.45 rad/s their noise standard
deviation doubles (a quarter of the weight), and the gyro leads during fast
motion.
::::

::::{dropdown} Keeping the quaternion on the unit sphere
:icon: unfold

After each step $q$ is renormalized. The covariance must be transformed
consistently. Differentiating $\hat q = q/\|q\|$ gives

$$
\frac{\partial \hat q}{\partial q} = \frac{I - \hat q\hat q^\top}{\|q\|},
$$

which projects out the radial direction. A unit quaternion has no
uncertainty in its length, so the covariance keeps no variance there
(`IMUEKF._normalize_quaternion_and_covariance`).
::::

::::{dropdown} Proof: why position and velocity can never be observed
:icon: unfold

A state is observable only if some measurement depends on it:

| State | Appears in | Observable? |
|---|---|---|
| roll, pitch | $h_g$ | yes, while `gravity_valid` |
| yaw | $h_m$ only | only with a magnetometer |
| $\mathbf b_g$ | through $q$ over time | partly |
| $\mathbf b_a$ | $h_g$ | partly (mixed with tilt) |
| **position, velocity** | **nothing** | **never** |

No measurement model contains $\mathbf p$ or $\mathbf v$, so their columns of
$H$ are zero. No correction ever reduces their variance; it only grows with
every prediction. Position is therefore *relative to the start* and drifts at
the rates derived in {doc}`imu-noise`. Bounding it needs an external position
or velocity source: GPS, optical flow, a barometer for altitude, or
zero-velocity updates.
::::

:::{admonition} Why it matters for the drone
:class: aerl-why
- In the 7 s demo, the EKF's position is already **6.5 cm** off, and the drift
  is accelerating (finding 6 on the {doc}`/findings/index` page).
- The live viewer therefore never shows the EKF position. It shows a "virtual
  flight" driven by attitude, and whenever the board is still it resets the
  velocity to zero and freezes the position (the crudest zero-velocity update,
  in {class}`~drone_sim.visualization.live_viewer.LiveOpenCRViewer`).
:::
