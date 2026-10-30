# IMU noise and drift

:::{admonition} In plain words
:class: aerl-plain
Every sensor reading is slightly wrong. Part of the error is random jitter
that changes every sample (**white noise**), and part is a small offset that
wanders slowly (**bias**). To get an angle, the gyroscope's readings are added
up over time, so these small errors **pile up**: the angle slowly drifts even
when the sensor sits perfectly still. Jitter makes the drift grow like √t.
Bias makes it grow in a straight line, and bias soon dominates. Position is
far worse, because the accelerometer is added up twice.
:::

```{localvideo} theory/v2_imu_noise.mp4
:title: IMU noise and drift
```

## The key idea

Each reading is modelled as the true value plus three kinds of error:

$$
\tilde y(t) = \underbrace{y(t)}_{\text{true}} + \underbrace{b_0}_{\text{fixed offset}}
+ \underbrace{b(t)}_{\text{wandering bias}} + \underbrace{n(t)}_{\text{white noise}} .
$$

Integrating a still gyro gives an angle error that grows like

$$
\sigma_\theta(t) \approx \sqrt{N^2 t + (B\,t)^2},
$$

| Symbol | Meaning |
|---|---|
| $N$ | white-noise density (rad/s/√Hz): sets the random-walk part, $N\sqrt t$ |
| $B$ | bias instability (rad/s): sets the straight-line part, $B\,t$ |
| $t$ | time since the angle was last correct (s) |
| $\sigma_\theta$ | typical size of the angle error (rad) |

**Worked numbers for the MPU6050** (measured by Gonzalez & Dabove, 2019):
after 7 s the white-noise part is only about **0.01°**. The bias part takes
over after about **17 s**.

```{figure} /_static/figures/gyro_drift.png
:width: 620px

200 simulated runs of a motionless MPU6050. The white noise term $N\sqrt t$
alone (red) explains only the first ~10 s; then the bias dominates, and the
combined formula (green) matches the simulated spread (black).
```

The cure is **sensor fusion**: let a drift-free reference (the direction of
gravity) slowly pull the gyro's estimate back, as in the figure below and on
the {doc}`madgwick` and {doc}`ekf` pages.

```{figure} /_static/figures/tilt_correction.png
:width: 620px

The same biased gyro (0.01 rad/s offset), integrated alone (blue) and fused
with gravity by the Madgwick filter (orange). Fusion keeps the error within
about 0.1°.
```

## Show the derivations

::::{dropdown} The full noise model and the measured MPU6050 parameters
:icon: unfold

| Term | Name | Parameter (per axis) | Behaviour |
|---|---|---|---|
| $y$ | true value | - | - |
| $b_0$ | constant bias | `*_constant_bias_*` | fixed offset, removable by calibration |
| $b(t)$ | bias instability | $B$, $\tau_c$ | wanders slowly; Gauss-Markov process |
| $n(t)$ | white noise | density $N$ | independent at every sample |

Measured values (Gonzalez & Dabove, 2019, *Table 2*):

| Axis | Gyro $N$ (rad/s/√Hz) | Gyro $B$ (rad/s) | Gyro $\tau_c$ (s) | Accel $N$ ((m/s²)/√Hz) | Accel $B$ (m/s²) |
|---|---|---|---|---|---|
| X | 6.625e-5 | 1.674e-5 | 900 | 1.156e-3 | 3.703e-4 |
| Y | 5.934e-5 | 2.301e-5 | 200 | 1.252e-3 | 2.501e-4 |
| Z | 6.050e-5 | 1.462e-5 | 200 | 1.820e-3 | 5.058e-4 |

**Why Gaussian?** Sensor noise is the sum of many small independent effects:
thermal noise, electronic noise, quantization. By the central limit theorem
their sum is approximately Gaussian, whatever each effect's own distribution.
The demo {mod}`~drone_sim.visualization.statistics_demo` shows it. Gaussian
noise is also what makes the Kalman filter optimal ({doc}`ekf`).
::::

::::{dropdown} Derivation: from noise density to per-sample noise (σ = N√fs)
:icon: unfold

A noise *density* $N$ describes continuous white noise with power spectral
density $N^2$. Sampled at $f_s$ (averaged over $\Delta t = 1/f_s$), each
sample has variance $N^2 / \Delta t$, so

$$
\sigma_n = N \sqrt{f_s}.
$$

At 100 Hz the MPU6050 gyro has $\sigma_n \approx 6.6\times10^{-4}$ rad/s per
sample. A higher sample rate gives noisier individual samples but the same
noise *per second*.
::::

::::{dropdown} Proof: the Gauss-Markov bias has spread exactly B
:icon: unfold

$b(t)$ obeys $\dot b = -b/\tau_c + w(t)$, a mean-reverting random walk.
Discretizing exactly over $\Delta t$:

$$
b_{k+1} = \varphi\, b_k + q\, w_k,\qquad \varphi = e^{-\Delta t/\tau_c},\qquad w_k \sim \mathcal N(0, 1).
$$

**Claim.** Choosing $q = B\sqrt{1 - \varphi^2}$ makes the stationary standard
deviation exactly $B$.

*Proof.* In steady state $\operatorname{Var}(b_{k+1}) = \operatorname{Var}(b_k) = \sigma^2$,
so $\sigma^2 = \varphi^2\sigma^2 + q^2$, giving $\sigma^2 = q^2/(1-\varphi^2) = B^2$. $\square$

{func}`~drone_sim.sensors.imu_simulator.gauss_markov_bias` implements this as
a linear filter and discards $5\tau_c$ of burn-in so the sequence starts in
steady state.
::::

::::{dropdown} Derivation: why integrated noise grows like √t, and bias like t
:icon: unfold

A gyro-only estimate is $\hat\theta(t) = \int_0^t \tilde\omega\,dt$, so its error
is the integral of the sensor error.

**White noise → angle random walk.** For $n = t f_s$ samples,

$$
\operatorname{Var}\!\big(\delta\theta(t)\big) = n\,\sigma_n^2\,\Delta t^2
= (t f_s)(N^2 f_s)\,\frac{1}{f_s^2} = N^2 t
\quad\Rightarrow\quad
\sigma_\theta(t) = N\sqrt t .
$$

The sample rate cancels. For the MPU6050 after 7 s,
$\sigma_\theta = 6.6\times10^{-5}\sqrt7 \approx 0.010^\circ$; a Monte Carlo run
of the simulator gives $0.0101^\circ$.

**Bias → linear drift.** Over times short compared with $\tau_c$, the bias is
effectively constant, and $\delta\theta = b\,t$ grows *linearly*. An
uncorrected 0.01 rad/s offset tilts the estimate by 17° in 30 s.

Combined, for $t \ll \tau_c$, $\sigma_\theta(t) \approx \sqrt{N^2 t + (B t)^2}$.
The two parts are equal at $t = N^2/B^2 \approx 17$ s for this gyro.
::::

::::{dropdown} Derivation: why position drifts so much faster
:icon: unfold

An IMU-only position estimate integrates the accelerometer *twice*:

| Error source | Velocity error | Position error |
|---|---|---|
| Accel white noise $N_a$ | $N_a\sqrt t$ | $N_a\, t^{3/2}/\sqrt3$ |
| Accel bias $b_a$ | $b_a\, t$ | $\tfrac12 b_a t^2$ |
| Tilt error $\delta\theta$ (gravity leaks in) | $g\,\delta\theta\, t$ | $\tfrac12 g\,\delta\theta\, t^2$ |

The last row is usually the largest: a $0.1^\circ$ tilt error leaks
$g \sin 0.1^\circ \approx 0.017$ m/s² of gravity into the horizontal axes,
about 0.9 m of position error after 10 s.
::::

:::{admonition} Why it matters for the drone
:class: aerl-why
- In the 7 s flight demo the gyro noise alone would cause only ~0.01° of
  error. The ~0.15° actually seen comes from the filter's integration step
  instead: a perfect, noise-free IMU gives the same error (finding 4 on the
  {doc}`/findings/index` page).
- The EKF's position is labelled *relative* because, as the table above
  shows, it drifts without bound. In the demo it is already 6.5 cm off after
  7 s.
:::
