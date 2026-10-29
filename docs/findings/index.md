# Findings

What building, breaking and testing this simulator revealed. Each finding
states the observation, the evidence (numbers from the project's own code),
the explanation, and what the code does about it. Several are **lessons
learned from bugs** found during development; they are kept here because
the way a bug was caught is as instructive as the fix.

Unless stated otherwise, numbers come from the 7 s figure-eight demo
(`run_pipeline()`, 500 Hz, seed 0).

---

## Attitude representation

### 1. Gimbal lock is unavoidable with three angles, so the filters use quaternions

**Observation.** As pitch approaches 90°, a constant, modest body rate of
$[1, 0.2, 0.5]$ rad/s produces roll and yaw *rates* of about 17,000 °/s (at
89.9°). At exactly 90°, the attitude depends on roll and yaw only through
$\psi - \phi$.

**Explanation.** The Euler-rate matrix has determinant $1/\cos\theta$, and at
$\theta = 90^\circ$ the rotation collapses to $R_z(\psi - \phi)R_y(90^\circ)$:
one degree of freedom is lost. This is not a flaw of the ZYX convention.
*No* three-parameter representation of rotations is singularity-free
(Stuelpnagel, 1964). Proofs: {doc}`/theory/euler-angles`.

**What the code does.** The plant (Euler angles) raises an error near 90° and
the controller limits tilt to 20°. Both estimators integrate **quaternions**,
whose kinematics $\dot q = \tfrac12 q\otimes(0,\boldsymbol\omega)$ have no
singularity. This matters in the live demo, where a person can point the
board straight up. Euler angles are used only for display.

```{figure} /_static/figures/gimbal_lock.png
:width: 560px
```

### 2. Lesson learned: two frame conventions need exactly one conversion

**Observation.** An early version had the plant (NED, z down) and the EKF
(z up) disagreeing about the sign of gravity and acceleration. The filters
were silently fighting a mirrored world.

**Explanation.** Both pairs of frames are related by
$S = \operatorname{diag}(1,-1,-1)$, a half turn about x. Because $\det S = +1$,
it is a *proper* rotation: vectors map as $Sv$ and quaternions as
$[w, x, -y, -z]$. A reflection would have flipped handedness and broken every
cross product.

**What the code does.** All conversions live in
{mod}`drone_sim.dynamics.frames`. A test rotates a vector both ways round and
checks that the two answers agree. See {doc}`/theory/frames`.

---

## Noise and drift

### 3. Gyro drift has two regimes: random walk, then bias

**Observation.** Integrating a motionless MPU6050 gyro, the spread of the
angle error over 200 runs follows $N\sqrt t$ for about 10 s, then grows
faster. After 7 s it is 0.010° (simulation: 0.0101°).

**Explanation.** White noise integrates to a random walk,
$\sigma = N\sqrt t$. The slowly wandering bias integrates to a *linear* drift,
$B\,t$. They cross at $t = N^2/B^2 \approx 17$ s. Derivation:
{doc}`/theory/imu-noise`.

```{figure} /_static/figures/gyro_drift.png
:width: 560px
```

### 4. In the demo, integration error beats sensor noise

**Observation.** The Madgwick attitude error peaks at 0.158° during the
fast recovery manoeuvre. With a **perfect, noise-free IMU** the peak is
still 0.158°. Raising the sample rate from 500 Hz to 2 kHz shrinks it to
0.039°, exactly 4× smaller.

| IMU | Rate | Peak error (roll) | Final error (roll) |
|---|---|---|---|
| MPU6050 noise | 500 Hz | 0.158° | −0.054° |
| noise-free | 500 Hz | 0.158° | −0.038° |
| noise-free | 2 kHz | 0.039° | −0.009° |

**Explanation.** The filters advance the quaternion with one explicit step,
$q + \dot q\,\Delta t$. Its error is first order in $\Delta t$ and grows with
the rotation rate, which reaches about 2.5 rad/s while the drone recovers
from its tilted start. The noise contributes only the difference between the first
two rows.

**What it means.** Before blaming the sensor, check the integrator. Higher
sample rates, or exact integration of the rotation over a step
(exponential map), would remove this error. The plant itself uses RK4, so the
*truth* is not affected.

---

## Sensor fusion

### 5. An accelerometer cannot tell tilt from acceleration

**Observation.** A level drone accelerating forward at $a$ reads
$[a, 0, g]$, the same direction as a stationary drone tilted by
$\arctan(a/g)$. A "gravity" correction during flight drags a correct
estimate wrong.

**Explanation.** An accelerometer measures *specific force*
$R^\top(\mathbf a - \mathbf g)$, not gravity ({doc}`/theory/frames`). Checking that
$\|\mathbf f\| \approx g$ catches shocks but not a sustained acceleration,
which can have exactly the norm of gravity.

**What the code does.** Both filters take an explicit `gravity_valid`
flag. The offline demo sets it False for the whole powered flight, so attitude
there is gyro-only (findings 3–4). The live viewer sets it True only when the
board is detected as stationary. A test feeds a constant non-vertical
acceleration and checks that the attitude does not move.

### 6. IMU-only position is unobservable and drifts without bound

**Observation.** The EKF position drifts 6.5 cm from the truth in 7 s, and the
drift accelerates. A tilt error of just 0.1° leaks
$g\sin 0.1^\circ \approx 0.017$ m/s² of gravity into the horizontal axes:
about 0.9 m of error after 10 s.

**Explanation.** No measurement model in the EKF contains position or
velocity, so their columns of $H$ are zero. Corrections can never reduce their
variance; it only grows. Error sources integrate twice: bias and tilt errors
give $t^2$ growth. See {doc}`/theory/ekf`.

**What the code does.** The position is labelled *relative* everywhere. The
live viewer does not display it at all: it shows a "virtual flight" driven by
attitude, and it freezes the EKF position while the board is still.

### 7. Lesson learned: yaw is unobservable without a magnetometer

**Observation.** An early version started both filters at "level, heading 0"
while the simulated drone started at yaw 0.5 rad. The filters showed a
constant 28.6° yaw error. It looked like a filter bug and was not one.

**Explanation.** Gravity is vertical, so rotating about the vertical leaves the
predicted accelerometer reading unchanged: the gravity term's gradient has no
yaw component. Without a magnetometer, heading can only be *propagated*,
never *measured*.

**What the code does.** The offline demo seeds both filters with the known
initial pose, and records this explicitly in the result's `metadata`. It is a
simulation reference, not an IMU capability.

### 8. Madgwick's "optimal" gain is very slow for a good gyro

**Observation.** Madgwick's rule $\beta = \sqrt{3/4}\,\tilde\omega$ gives
$\beta \approx 5.4\times10^{-4}$ for the MPU6050 at 100 Hz. That corrects
attitude at only $2\beta \approx 0.06^\circ$/s: almost 3 minutes to remove a
10° initial error.

**Explanation.** The rule matches the correction speed to the gyro's error
rate, which assumes a well-initialized filter. A low-noise gyro gives a tiny
$\beta$. Madgwick's own reference implementation defaults to $\beta = 0.1$.

**What the code does.** The demo initializes from the known pose, so the slow
gain never matters there. The fusion figure in {doc}`/theory/imu-noise` uses
$\beta = 0.05$ to show convergence.

### 9. Lesson learned: the OpenCR library reported acceleration 4× too large

**Observation.** On the real board, a resting accelerometer read about 4 g.

**Explanation.** OpenCR library 1.5.3 converts raw counts with the ±8 g scale
(`8/32768`), while both bundled IMU drivers configure the sensor for ±2 g.
Every reading was multiplied by 8/2 = 4.

**What the code does.** The firmware multiplies by 2/8 and documents why.
The live viewer's startup calibration recognizes streams that are off by
exactly ×4 or ×¼ (older firmware) and corrects them, and it reports any other
magnitude as a visible calibration failure instead of silently rescaling.
See {doc}`/hardware`.

---

## Dynamics and control

### 10. The controller asks for torques the motors cannot produce

**Observation.** On the first step of the demo, the rate loop requests
$\boldsymbol\tau = [-2.50, 0.38, -3.21]$ N·m. The motors can deliver only about a
tenth of that while still producing the needed 11.8 N of thrust:
$[-0.26, 0.04, -0.34]$ N·m.

**Explanation.** Torque comes from *differences* in motor thrust, and each
motor's thrust lies between 0 and $k_t\omega_{\max}^2$. The project's review
found that simply clipping each motor (the original approach) turned a
similar first command's 12.6 N of thrust into 52 N, enough to launch the drone
upward.

**What the code does.**
{meth}`~drone_sim.dynamics.plant.DronePlant.allocate_control` keeps the thrust
and scales the torque *uniformly*, preserving its direction, to the largest
feasible value. Holding altitude takes priority over turning quickly. In the
demo a motor hits zero speed on about 2 % of steps. See {doc}`/theory/dynamics`.

### 11. LQR outside its linearization: robust, but not optimal

**Observation.** In the recovery demo the LQR pitches the drone to about 58°
(1.01 rad), far from the hover point it was designed around, and still
converges in about 5 s. On the moving figure-eight it tracks 2–10× worse
than the PID:

| Controller | RMSE x | RMSE y | RMSE altitude |
|---|---|---|---|
| Cascaded PID | 0.139 m | 0.163 m | 0.011 m |
| LQR (hover) | 0.330 m | 0.497 m | 0.116 m |

**Explanation.** $K$ is optimal only for the *linearized* model near hover,
and for regulation to a fixed point, not tracking a moving one. LQR's
guaranteed stability margins explain why it survives large angles. Its
optimality does not carry over. See {doc}`/theory/lqr`.

### 12. Lesson learned: every RK4 stage needs the full trial state

**Observation.** An early plant evaluated the derivative at the trial states
for position and angles, but used the *current* body rates in every stage, so
the body rates were never actually integrated by RK4. The fix is checked
against the closed-form answer for a constant torque, $\omega = \alpha t$ and
$\psi = \tfrac12\alpha t^2$.

**Explanation.** RK4's fourth-order accuracy comes from evaluating $f$ at
$\mathbf x + \tfrac h2 k_1$, $\mathbf x + \tfrac h2 k_2$, $\mathbf x + h k_3$. Freezing
part of the state turns the method into a hybrid of RK4 and Euler.

**What the code does.**
{meth}`~drone_sim.dynamics.plant.DronePlant.state_derivative` is a pure
function of the full trial state. The test
`test_rk4_integrates_angular_acceleration_and_rate` checks the closed-form
answer to 10 decimal places.

---

## Caveats

### 13. Good tracking says nothing about the filters

The controllers fly using the plant's **true** state; the filters run
afterwards on the simulated IMU stream. The tracking plots therefore validate
the controller and the plant, and the estimation plots validate the filters,
but nothing here shows the drone flying on its own estimates. That would be
the next step: closing the loop through the EKF.

### 14. Multi-rate loops must use the actual elapsed time

The position loop runs at 50 Hz and the attitude loop at 500 Hz on top of the
simulation step. When a period does not divide evenly into simulation steps,
passing the *nominal* period to a PID makes its integral and derivative
slightly wrong on every update. The scheduler passes the actual accumulated
time (`test_controller_scheduler_passes_accumulated_elapsed_time`).
