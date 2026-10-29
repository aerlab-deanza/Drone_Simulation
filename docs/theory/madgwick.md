# The Madgwick filter

:::{admonition} In plain words
:class: aerl-plain
The Madgwick filter follows the gyroscope, which is smooth but drifts, and
keeps **gently nudging** the estimate so that "down" agrees with what the
accelerometer sees. A magnetometer can do the same for "north". The size of
each nudge is set by one number, **β**: bigger corrects faster but lets more
accelerometer noise in. One catch: gravity can correct roll and pitch, but it
says nothing about which way the drone is *facing*.
:::

```{localvideo} theory/v3_fusion.mp4
:title: Sensor fusion: Madgwick and the EKF
```

## The key idea

Each sample, the quaternion changes by the gyro's prediction **minus** a small
correction step:

$$
\dot q = \underbrace{\tfrac12\, q \otimes (0, \boldsymbol\omega)}_{\text{follow the gyro}}
\;-\; \underbrace{\beta \,\frac{\nabla f}{\|\nabla f\|}}_{\text{nudge toward "down"}} .
$$

| Symbol | Meaning |
|---|---|
| $q$ | current attitude estimate (a quaternion) |
| $\boldsymbol\omega$ | gyro reading (rad/s) |
| $f$ | mismatch between the predicted "up" and the accelerometer's measured "up" |
| $\nabla f / \|\nabla f\|$ | direction that reduces the mismatch fastest (unit length) |
| $\beta$ | correction speed (a rotation of $2\beta$ rad/s) |

Picture the mismatch as a landscape over all possible attitudes. The filter
takes **one small step downhill** per sample, toward the attitude whose "up"
matches the accelerometer.

**Worked numbers:** Madgwick's recommended β for the MPU6050 is only about
$5.4\times10^{-4}$, which is a correction of **0.06° per second**. A filter
started 10° off would need almost **3 minutes** to converge. Madgwick's own
reference code uses β = 0.1.

In code: {class}`~drone_sim.estimation.madgwick.MadgwickMARG`.

## Show the derivations

::::{dropdown} The objective function and its gradient
:icon: unfold

With $q$ mapping body to world (z up), world "up" seen in body axes is
$R(q)^\top \hat z$, the third row of $R(q)$. If the body is not accelerating,
the normalized accelerometer reading $\hat{\mathbf a}$ should equal it. The
residual is

$$
\mathbf f_g(q) = R(q)^\top \hat z - \hat{\mathbf a} =
\begin{bmatrix}
2(q_2 q_4 - q_1 q_3) - \hat a_x\\
2(q_1 q_2 + q_3 q_4) - \hat a_y\\
1 - 2(q_2^2 + q_3^2) - \hat a_z
\end{bmatrix},
$$

with $q = [q_1, q_2, q_3, q_4] = [w, x, y, z]$. The magnetometer adds three
rows $\mathbf f_b(q) = R(q)^\top [b_x, 0, b_z]^\top - \hat{\mathbf m}$
({meth}`~drone_sim.estimation.madgwick.MadgwickMARG.objective_function`).

Minimizing $\tfrac12\|\mathbf f\|^2$ gives the gradient

$$
\nabla \tfrac12\|\mathbf f\|^2 = J^\top \mathbf f,
\qquad J = \frac{\partial \mathbf f}{\partial q},
$$

where $J$ is written out analytically in
{meth}`~drone_sim.estimation.madgwick.MadgwickMARG.objective_jacobian`. The
full update then renormalizes:
$q \leftarrow (q + \dot q\,\Delta t)/\|q + \dot q\,\Delta t\|$.
::::

::::{dropdown} Proof: gravity alone cannot fix yaw
:icon: unfold

Rotating about the vertical leaves $R^\top\hat z$ unchanged, so $\mathbf f_g$
is constant along yaw and its gradient has no yaw component. Heading needs
the magnetometer, or it drifts like a pure gyro.
::::

::::{dropdown} Derivation: choosing β = √(3/4) ω̃
:icon: unfold

Madgwick sets $\beta$ to match the rate at which gyro error corrupts $q$. If
each gyro axis has error $\tilde\omega$, then

$$
\beta = \Big\|\tfrac12\, q \otimes (0, \tilde\omega, \tilde\omega, \tilde\omega)\Big\|
= \tfrac12\sqrt3\,\tilde\omega = \sqrt{\tfrac34}\,\tilde\omega.
$$

{func}`~drone_sim.estimation.madgwick.default_beta` computes $\tilde\omega$ from
the MPU6050 table: the per-sample white noise $N\sqrt{f_s}$ combined with the
bias instability $B$. At 100 Hz this gives $\beta \approx 5.4\times10^{-4}$, so
$2\beta \approx 1.1\times10^{-3}$ rad/s ≈ $0.06^\circ$/s. This β is "optimal"
only for a perfectly initialized filter on an excellent gyro.
::::

::::{dropdown} Gyro bias estimation (ζ) and magnetic distortion compensation
:icon: unfold

**Bias.** The normalized gradient direction, mapped back into a body rate, is
an estimate of the gyro's error:

$$
\boldsymbol\omega_\varepsilon = 2\, \hat q^* \otimes \frac{J^\top\mathbf f}{\|J^\top\mathbf f\|}.
$$

Integrating it slowly, $\mathbf b_\omega \leftarrow \mathbf b_\omega + \zeta\,\boldsymbol\omega_\varepsilon\Delta t$,
and subtracting $\mathbf b_\omega$ from the gyro removes a slowly varying bias.
$\zeta = 0$ disables this.

**Magnetic distortion.** Indoors, the local field's inclination is unknown and
distorted. After each update the measurement is rotated into the world frame,
$\mathbf h = q \otimes (0, \hat{\mathbf m}) \otimes q^*$, and the reference is
rebuilt as $b_x = \sqrt{h_x^2 + h_y^2}$, $b_z = h_z$
({meth}`~drone_sim.estimation.madgwick.MadgwickMARG.update_magnetic_reference`).
The magnetometer can then only correct heading, never roll or pitch.
::::

:::{admonition} Why it matters for the drone
:class: aerl-why
- During powered flight the accelerometer also measures thrust and
  acceleration, so its "down" is wrong ({doc}`frames`). The offline demo
  therefore sets `gravity_valid=False` for the whole flight, and the filter
  follows the gyro only. A test checks that a constant non-vertical
  acceleration leaves the attitude untouched.
- Because the recommended β is so slow, the demo starts the filter from the
  known attitude. The fusion figure on {doc}`imu-noise` uses β = 0.05 to show
  convergence.
:::
