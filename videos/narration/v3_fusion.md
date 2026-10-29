# Video 3 — Sensor fusion: Madgwick and the EKF

Target length: about 6 minutes. One audio file per scene, named by scene
ID, in `videos/audio/v3_fusion/`.

---

## S01Intro (≈ 20 s)

**On screen:** gyro estimate drifting vs accelerometer estimate noisy;
the two combine; title.

> The gyroscope gives smooth, precise attitude over short times, but it
> drifts. The accelerometer knows which way is down, and never drifts, but
> it's noisy. Sensor fusion takes the best of both. We'll look at two ways to
> do it — the Madgwick filter and the extended Kalman filter — and at what
> neither of them can ever know.

## S02SpecificForce (≈ 40 s)

**On screen:** f = Rᵀ(a − g); four cases (table, free fall, hover,
accelerating); finding 5: a tilted, still box and a level, accelerating box
give the same reading.

> First, what does an accelerometer actually measure? Not gravity — gravity
> pulls the sensor's proof mass and its case equally. It measures everything
> else: the specific force, R-transpose times acceleration minus gravity.
> Resting on a table, it reads plus g, upward — that's the table pushing.
> In free fall, it reads zero. Hovering, thrust replaces the table. And here's
> the trap: a level drone accelerating forward reads the same direction as a
> drone that's standing still but tilted. The accelerometer alone cannot tell
> tilt from acceleration.

## S03Objective (≈ 40 s)

**On screen:** predicted up Rᵀẑ vs measured â; the residual f(q); its
gradient Jᵀf; gradient descent on the error surface over roll and pitch.

> Madgwick's idea: when the body isn't accelerating, the predicted direction
> of "up", seen from the body, should match the measured accelerometer
> direction. The difference is a residual, f of q. Its squared length is an
> error surface over all attitudes, and its gradient — J-transpose times f —
> points uphill. So each sample, we take one small step downhill. Here's
> that surface for our simulator's objective function, and the path the
> estimate follows towards the true tilt.

## S04FusedUpdate (≈ 40 s)

**On screen:** q̇ = q̇_ω − β∇f/‖∇f‖; β = √(3/4) ω̃; finding 8: β ≈ 5.4e-4,
0.06°/s, almost 3 minutes for 10°.

> The full update follows the gyroscope, minus beta times the normalized
> gradient. Because the gradient is normalized, beta is literally the speed of
> correction. Madgwick sets it to match how fast gyro error corrupts the
> quaternion: root three over two times the gyro's error. For our excellent
> MPU-6050, that's only about five times ten to the minus four — a correction
> of six hundredths of a degree per second. A filter that started ten
> degrees off would need almost three minutes to converge. That's why our
> demo starts the filter from the known attitude.

## S05YawUnobservable (≈ 35 s)

**On screen:** finding 7. A box spins about the vertical; the gravity arrow
in body axes doesn't change; 28.6° constant yaw error; magnetometer.

> Now spin the body about the vertical axis. Gravity still points straight
> down, so the accelerometer reading doesn't change at all — and neither does
> the residual. The gradient has no yaw component. Without a magnetometer,
> heading can be propagated by the gyro, but never measured. In an early
> version of our simulator, the filter started at heading zero while the drone
> started at about twenty-nine degrees. The constant twenty-nine-degree error
> looked like a filter bug. It wasn't — yaw was simply unobservable.

## S06EKFPredict (≈ 40 s)

**On screen:** the 16-element state; strapdown prediction equations;
an uncertainty ellipse that grows.

> The extended Kalman filter takes a different approach: it tracks not just
> the state, but how uncertain it is. The state has sixteen numbers —
> position, velocity, the quaternion, and gyro and accelerometer biases. Each
> IMU sample predicts the next state: the gyro turns the quaternion, the
> accelerometer, rotated into the world with gravity removed, moves velocity
> and position. And the covariance — the uncertainty — is pushed through the
> linearized dynamics. With every prediction, it grows.

## S07EKFUpdate (≈ 40 s)

**On screen:** K = P Hᵀ (H P Hᵀ + R)⁻¹; the ellipse shrinks along the
measured direction; Joseph form as a sum of two PSD terms.

> When a measurement arrives — the gravity direction — the filter compares it
> with its prediction. The Kalman gain decides how much to trust each: a
> confident prediction moves little, an uncertain one moves a lot. The
> uncertainty shrinks — but only in the directions the measurement can see.
> Our filter updates the covariance with the Joseph form: two terms that are
> each guaranteed positive, so rounding can never produce an impossible,
> negative uncertainty.

## S08Observability (≈ 40 s)

**On screen:** finding 6. Table of states vs which measurement sees them;
position and velocity columns of H are zero; the EKF's position error from
the demo, growing to 6.5 cm.

> So which states can the IMU actually see? Roll and pitch: yes, through
> gravity. Yaw: only with a magnetometer. Position and velocity: never. No
> measurement in the filter depends on them, so their columns in the
> measurement matrix are zero, and their uncertainty can only grow. In our
> seven-second demo, the EKF position drifts six and a half centimetres —
> and the drift is accelerating. Bounding it needs an outside reference:
> GPS, a camera, optical flow.

## S09GravityValid (≈ 35 s)

**On screen:** the gravity_valid rules (offline: off; live: only when still);
noise inflated as 1 + (ω/0.45)²; finding 9: the OpenCR 4× scaling story.

> That's why both filters take a flag, gravity-valid, set by the caller. In
> the offline flight it's off the whole time; in the live demo, it's on only
> when the board is still. During fast turns, the filter also trusts the
> reference vectors less. And one hardware lesson: on the real OpenCR board,
> a resting accelerometer first read four g, because the board's library
> scaled for plus-or-minus eight g while the chip was set to two. The
> firmware now corrects it — and the viewer checks for one g before it
> calibrates.

## S10Outro (≈ 15 s)

**On screen:** summary bullets; "Next: dynamics and control".

> Gyros for speed, gravity for truth, and a clear view of what can't be
> observed. In the final video: the physics of the drone itself, and the
> controllers that fly it.
