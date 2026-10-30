# Video 3 - Sensor fusion: Madgwick and the EKF (on-screen captions)

One bullet = one caption card, shown in order across the scene.

## S01Intro
- The gyro gives smooth, precise attitude over short times, but it drifts.
- The accelerometer knows which way is down and never drifts, but it is noisy.
- Sensor fusion takes the best of both. Two ways to do it: the Madgwick filter and the extended Kalman filter.

## S02SpecificForce
- What does an accelerometer measure? Not gravity: gravity pulls the sensor and its case equally.
- It measures specific force: f = Rᵀ(a − g).
- On a table it reads +g upward (the table pushing). In free fall it reads 0. Hovering, thrust replaces the table.
- The trap: a level drone accelerating forward reads the same as a still drone that is tilted.
- From the accelerometer alone, tilt and acceleration cannot be told apart.

## S03Objective
- Madgwick's idea: when the body is not accelerating, the predicted "up" should match the measured direction.
- The difference is a residual f(q). Its squared length is an error surface over all attitudes.
- Its gradient Jᵀf points uphill, so each sample takes one small step downhill.
- This is that surface for the simulator's objective function, and the path the real filter follows to the true tilt.

## S04FusedUpdate
- The update follows the gyro, minus β times the normalized gradient. So β is the speed of correction.
- Madgwick sets β = √(3/4)·ω̃ to match how fast gyro error corrupts the quaternion.
- For the excellent MPU6050 that is only ~5.4 × 10⁻⁴: a correction of 0.06° per second.
- A filter started 10° off would need almost 3 minutes to converge.
- That is why the demo starts the filter from the known attitude.

## S05YawUnobservable
- Spin the body about the vertical axis: gravity still points straight down…
- …so the accelerometer reading, and the residual, do not change. The gradient has no yaw part.
- Without a magnetometer, heading can be propagated by the gyro but never measured.
- Lesson learned: a constant 28.6° yaw error in an early version looked like a filter bug. Yaw was simply unobservable.

## S06EKFPredict
- The extended Kalman filter tracks not only the state but also how uncertain it is.
- The state has 16 numbers: position, velocity, quaternion, gyro bias and accelerometer bias.
- Each IMU sample predicts the next state: the gyro turns q; the accelerometer, minus gravity, moves v and p.
- The covariance P is pushed through the linearized dynamics. With every prediction, uncertainty grows.

## S07EKFUpdate
- A gravity measurement arrives. The filter compares it with its prediction.
- The Kalman gain K decides whom to trust: a confident prediction moves little, an uncertain one moves a lot.
- Uncertainty shrinks, but only in the directions the measurement can see.
- The Joseph-form update is a sum of two positive terms, so rounding can never make P impossible.

## S08Observability
- Which states can an IMU see? Roll and pitch: yes, through gravity.
- Yaw: only with a magnetometer.
- Position and velocity: never. No measurement depends on them, so their uncertainty only grows.
- In the 7 s demo the EKF position drifts 6.6 cm, and the drift is accelerating.
- Bounding it needs an outside reference: GPS, a camera, optical flow.

## S09GravityValid
- That is why both filters take a gravity_valid flag, set by the caller.
- Offline flight: off the whole time. Live demo: on only while the board is still.
- During fast turns, the filter also trusts the reference vectors less.
- Hardware lesson: the OpenCR library scaled for ±8 g while the chip was set to ±2 g, so a resting board read 4 g.
- The firmware now corrects it, and the viewer checks for 1 g before it calibrates.

## S10Outro
- Gyros for speed, gravity for truth, and a clear view of what cannot be observed.
- Next: the physics of the drone, and the controllers that fly it.
