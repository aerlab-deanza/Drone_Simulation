# Video 1 - Euler angles, gimbal lock, and quaternions

Target length: about 5 minutes. Record **one audio file per scene**, named
exactly as the scene ID (e.g. `S01Intro.wav`), into `videos/audio/v1_rotations/`.
Speak at a relaxed pace; the animation is stretched to fit each clip.

Pronunciation: φ "fee" (roll), θ "THAY-ta" (pitch), ψ "sigh" (yaw).

---

## S01Intro (≈ 20 s)

**On screen:** a quadcopter tumbles in 3D; title "Euler angles, gimbal lock,
and quaternions".

> A drone's flight controller has to answer one question hundreds of times a
> second: which way am I pointing? The obvious answer is three angles - roll,
> pitch and yaw. In this video we'll see why those three angles break down
> at one particular attitude, prove it, and see what our simulator uses
> instead.

## S02EulerAngles (≈ 35 s)

**On screen:** the drone rotates by yaw, then pitch, then roll; each
rotation matrix appears; they combine into R.

> Euler angles describe an attitude as three rotations in sequence. First
> yaw, psi, about the vertical axis. Then pitch, theta, about the new
> sideways axis. Then roll, phi, about the new forward axis. Each one is a
> simple rotation matrix, and multiplying them gives the full body-to-world
> rotation: R equals R-z of psi, times R-y of theta, times R-x of phi.
> In our code that's the function body-to-world-rotation.

## S03BodyRates (≈ 40 s)

**On screen:** three rate arrows on the drone; the matrix W assembles
column by column.

> But a gyroscope doesn't measure how fast the Euler angles change. It
> measures the body's angular velocity, omega, in its own axes. To connect
> them, we add up three rotation rates, each expressed in body axes. Roll
> rate acts directly about the body x axis. Pitch rate acts about an axis that
> the roll has already turned. And yaw rate acts about the world vertical,
> seen through both later rotations. Collecting terms gives a matrix W:
> omega equals W times the Euler-angle rates. The simulator needs the
> opposite direction, so it inverts W.

## S04RateBlowUp (≈ 35 s)

**On screen:** determinant expansion → det W = cos θ; graph of Euler rates
blowing up near 90°, with the project's 17,000 °/s value.

> Here's the first proof that something goes wrong. Expand the determinant
> of W along its first column. The sines and cosines of roll combine into one,
> and what's left is simply cosine theta. So W stops being invertible when
> pitch reaches ninety degrees. The inverse contains one over cosine theta.
> In our simulator, a perfectly ordinary body rate of one radian per second
> turns into seventeen thousand degrees per second of roll and yaw rate,
> just a tenth of a degree from vertical.

## S05GimbalLock (≈ 45 s)

**On screen:** three nested gimbal rings; pitch goes to 90°, the inner and
outer axes line up; the algebra R = Rz(ψ−φ)Ry(90°) appears.

> The second proof explains the name. Picture three nested gimbals: yaw on the
> outside, pitch in the middle, roll on the inside. Rotate the pitch gimbal
> to ninety degrees, and the roll axis swings round until it lines up with
> the yaw axis. Now two of the three gimbals turn the object the same way, and
> one direction of rotation is simply gone. Algebraically, conjugating the
> roll rotation by a quarter-turn pitch turns it into a rotation about the
> vertical. The whole matrix collapses to R-z of psi minus phi, times the
> pitch. Only the difference of yaw and roll survives. That is gimbal lock.

## S06NoThreeParameter (≈ 20 s)

**On screen:** "Every 3-parameter representation has a singularity" -
Stuelpnagel, 1964; 3 → 4 numbers.

> Is this just a bad choice of angle order? No. It's a theorem: no set of
> three parameters can describe every rotation without a singularity
> somewhere. The space of rotations simply isn't shaped like ordinary
> three-dimensional space. The fix is to use four numbers with one
> constraint.

## S07Quaternions (≈ 50 s)

**On screen:** q = (cos α/2, sin α/2 n); the Hamilton product; the
sandwich product rotating a vector; Rodrigues' formula.

> Those four numbers form a unit quaternion. A rotation by angle alpha about
> the axis n is the quaternion cos of alpha over two, plus sine of alpha over
> two times n. Quaternions multiply with the Hamilton product, which
> includes a cross product - so, like rotations, the order matters. To rotate
> a vector, we sandwich it: q, times the vector, times the conjugate of q.
> Expanding that product and using the half-angle identities gives exactly
> Rodrigues' rotation formula. So a unit quaternion really is a rotation.

## S08QuaternionKinematics (≈ 35 s)

**On screen:** q̇ = ½ q ⊗ (0, ω); "no tan θ, no 1/cos θ"; the norm
growing in one discrete step; renormalization.

> And here's the payoff. The gyroscope's angular velocity updates the
> quaternion through q-dot equals one half q times omega. There's no tangent,
> no division by cosine - it's well-behaved for every attitude. The exact
> equation keeps q at unit length. But one discrete step, q plus q-dot times
> delta-t, makes it slightly longer. So after every update, our filters
> divide q by its length.

## S09InTheProject (≈ 30 s)

**On screen:** a table: plant (Euler, guarded), controller (±20°), filters
(quaternions), live demo (board pointed straight up); then the frame
lesson S = diag(1, −1, −1).

> In the simulator, the plant still uses Euler angles: it flies far from
> vertical, and refuses to continue if it ever gets close. The controller
> limits tilt to twenty degrees. But both estimators use quaternions,
> because in the live demo a person can point the board straight up. One
> more lesson from development: the plant uses north-east-down axes and the
> filters use z-up. Converting between them is one half-turn, S. Keeping
> that conversion in a single place fixed a real sign bug.

## S10Outro (≈ 10 s)

**On screen:** summary bullets and the docs link.

> Euler angles for intuition, quaternions for computation. Next: why a
> gyroscope's angle drifts even when it's sitting perfectly still.
