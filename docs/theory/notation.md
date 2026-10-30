# Notation and glossary

A quick reference for the symbols and terms used on the theory pages.

## Symbols

| Symbol | Meaning | Unit |
|---|---|---|
| $\phi, \theta, \psi$ | roll, pitch, yaw: the three Euler angles (ZYX order) | rad |
| $\dot\phi, \dot\theta, \dot\psi$ | how fast each Euler angle changes | rad/s |
| $\boldsymbol\omega = [\omega_x, \omega_y, \omega_z]$ | body spin rate, what a gyroscope measures | rad/s |
| $R$ | rotation matrix from the body frame to the world frame | - |
| $R^\top$ | its transpose: from world back to body | - |
| $q = (w, x, y, z)$ | unit quaternion (attitude), scalar part first | - |
| $q^*$ | quaternion conjugate, the reverse rotation | - |
| $\otimes$ | quaternion multiplication (order matters) | - |
| $\mathbf p$, $\mathbf v$, $\mathbf a$ | position, velocity, acceleration | m, m/s, m/s² |
| $\mathbf f$ | specific force, what an accelerometer measures | m/s² |
| $g$ | gravity: 9.81 in the plant, 9.80665 for the hardware | m/s² |
| $m$, $I$ | mass and moment of inertia | kg, kg·m² |
| $T$ | total propeller thrust | N |
| $\boldsymbol\tau$ | torques about the body axes | N·m |
| $\omega_1 \dots \omega_4$ | the four motor speeds | rad/s |
| $N$ | white-noise density of a sensor | (unit)/√Hz |
| $B$, $\tau_c$ | bias instability and its correlation time | (unit), s |
| $\beta$ | Madgwick correction speed | rad/s |
| $P$ | uncertainty (covariance) in the EKF; Riccati solution in the LQR | - |
| $K$ | Kalman gain (EKF) or feedback gain (LQR) | - |
| $Q$, $R$ | LQR weights on state error and on input effort | - |
| $\Delta t$, $h$ | time step | s |

A dot above a symbol means "rate of change": $\dot x = dx/dt$.

## Frames

NED / FRD
: The physics model's frames. World: x North, y East, z **Down**. Body: x
  Forward, y Right, z **Down**. Standard in aerospace.

z-up / FLU
: The filters' and the sensor's frames. World: z **Up**. Body: x Forward,
  y Left, z **Up**.

$S = \operatorname{diag}(1,-1,-1)$
: The half-turn about x that converts between the two. See {doc}`frames`.

## Terms

Attitude
: The drone's orientation: which way it is pointing.

Specific force
: Acceleration minus gravity, seen from the body. An accelerometer measures
  this, not gravity. See {doc}`frames`.

Gimbal lock
: The pitch of ±90°, where roll and yaw merge and one rotation direction can
  no longer be described with Euler angles. See {doc}`euler-angles`.

Bias / drift
: A slowly changing sensor offset. Integrating it makes estimates wander away
  over time. See {doc}`imu-noise`.

Sensor fusion
: Combining a fast but drifting sensor (the gyro) with a slow but drift-free
  reference (gravity, north). See {doc}`madgwick` and {doc}`ekf`.

Observable
: A state the measurements can actually correct. Position is not observable
  from an IMU alone. See {doc}`ekf`.

RK4
: Fourth-order Runge-Kutta, the method that steps the physics forward in time.
  See {doc}`dynamics`.

Linearization
: Approximating the dynamics by a straight-line (linear) model near one
  operating point, here hover. See {doc}`lqr`.
