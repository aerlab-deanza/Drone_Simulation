# Video 1 - Euler angles, gimbal lock, and quaternions (on-screen captions)

One bullet = one caption card, shown in order across the scene.

## S01Intro
- A flight controller must answer one question hundreds of times a second: which way am I pointing?
- The obvious answer is three angles: roll, pitch and yaw.
- This video shows where those angles break down, proves it, and shows what the simulator uses instead.

## S02EulerAngles
- Euler angles describe an attitude as three rotations in sequence.
- First yaw ψ about the vertical z axis…
- …then pitch θ about the new y axis…
- …then roll φ about the newest x axis.
- Multiplying the three matrices gives the body-to-world rotation R = Rz(ψ) Ry(θ) Rx(φ).
- In the code: drone_sim.dynamics.plant.body_to_world_rotation().

## S03BodyRates
- A gyroscope does not measure how fast the Euler angles change.
- It measures the body's angular velocity ω, in the body's own axes.
- To connect them, add the three rotation rates, each expressed in body axes.
- Roll rate acts about body x directly. Pitch rate is seen through the roll. Yaw rate is seen through both.
- Collecting terms gives a matrix W: ω = W(φ, θ) × (the Euler-angle rates).
- The plant needs the opposite direction, so it uses W⁻¹.

## S04RateBlowUp
- Proof 1: expand det W along its first column.
- The roll terms combine into cos²φ + sin²φ = 1, leaving det W = cos θ.
- So W cannot be inverted at θ = ±90°, and W⁻¹ contains 1 / cos θ.
- In the simulator, a constant body rate of about 1 rad/s becomes ~17,000 °/s of roll and yaw rate at 89.9° pitch.

## S05GimbalLock
- Proof 2 explains the name. Picture three nested gimbals: yaw outside, pitch in the middle, roll inside.
- Turning the pitch gimbal to 90° swings the roll axis until it lines up with the yaw axis.
- Now two gimbals turn the object the same way: one direction of rotation is gone.
- Algebraically, a quarter-turn of pitch turns the roll rotation into a rotation about the vertical.
- The whole matrix collapses to R = Rz(ψ − φ) Ry(90°). Only ψ − φ survives. That is gimbal lock.

## S06NoThreeParameter
- Is this just a bad choice of angle order? No.
- It is a theorem: no three-parameter representation covers every rotation without a singularity.
- The fix: four numbers with one constraint.

## S07Quaternions
- Those four numbers form a unit quaternion: q = (cos α/2, sin α/2 · n) rotates by α about axis n.
- Quaternions multiply with the Hamilton product, which contains a cross product…
- …so, like rotations, the order matters: i ⊗ j = k but j ⊗ i = −k.
- To rotate a vector u, sandwich it: q ⊗ (0, u) ⊗ q*.
- Expanding with half-angle identities gives exactly Rodrigues' rotation formula. A unit quaternion is a rotation.

## S08QuaternionKinematics
- The gyro updates the quaternion through dq/dt = ½ q ⊗ (0, ω).
- No tan θ, no 1 / cos θ: it is well-behaved for every attitude.
- The exact equation keeps ‖q‖ = 1…
- …but one discrete step q + (dq/dt)·Δt makes q slightly longer.
- So after every update, the filters divide q by its length.

## S09InTheProject
- The plant still uses Euler angles: it flies far from vertical and stops with an error if it gets close.
- The PID controller limits tilt to ±20°.
- Both estimators use quaternions, because in the live demo the board can point straight up.
- Lesson learned: the plant uses NED/FRD axes, the filters use z-up/FLU.
- One half-turn S = diag(1, −1, −1) converts between them. Keeping it in one module fixed a real sign bug.

## S10Outro
- Euler angles for intuition, quaternions for computation.
- Next: why a gyroscope's angle drifts even when it is sitting perfectly still.
