# Video 4 - Dynamics and control: RK4, PID and LQR

Target length: about 6.5 minutes. One audio file per scene, named by scene
ID, in `videos/audio/v4_control/`.

---

## S01Intro (≈ 20 s)

**On screen:** side view of the drone recovering from a tilted, spinning
start under LQR control; title.

> Our simulated drone starts five metres off target, tilted, and spinning -
> and recovers in a few seconds. To make that happen we need three things: a
> model of the physics, a way to integrate it accurately, and a controller.
> This video derives all three.

## S02NewtonEuler (≈ 40 s)

**On screen:** Newton's law for translation; Euler's equation
τ = Iω̇ + ω × Iω; its three components; the gyroscopic coupling term
highlighted.

> Translation is Newton's second law: thrust, rotated into the world, plus
> drag, divided by mass, plus gravity. Rotation is Euler's equation. Angular
> momentum is I times omega in body axes - but the body axes themselves
> rotate, so the torque equals I omega-dot plus omega cross I omega. That
> cross product is the gyroscopic term: spinning about one axis while pitching
> produces a roll acceleration. For a symmetric quadcopter, it gives three
> simple equations, exactly as in our plant model.

## S03MotorMixing (≈ 40 s)

**On screen:** top view of the X frame; each motor's force and lever arm;
r × F; the mixing matrix M; spin directions.

> Four motors produce everything. Each pushes along the body's minus-z axis
> with force k-t omega squared, and resists its own spin with a torque k-b
> omega squared. In an X frame each motor sits forty-five degrees off the
> axes, so its lever arm is the arm length over root two. The torque of each
> force is r cross F. Collecting all four gives one matrix, M, from the four
> squared motor speeds to thrust and three torques. Two motors spin clockwise,
> two counter-clockwise - that's what lets the drone control yaw.

## S04Saturation (≈ 40 s)

**On screen:** finding 10. Requested vs feasible torque; the allocation
s = s_T + λ s_τ with λ chosen to fit the motor limits; thrust preserved.

> M is invertible, so any request has a solution - on paper. But motors can't
> spin backwards, and they have a top speed. On the first step of our demo,
> the controller asks for about ten times more torque than the motors can
> produce while holding the drone up. Simply clipping each motor turned a
> similar request's twelve newtons of thrust into fifty-two. So our allocator
> keeps the thrust, and scales the torque down uniformly, by the largest
> factor lambda that keeps every motor feasible. Holding altitude beats
> turning fast.

## S05RK4 (≈ 45 s)

**On screen:** the four RK4 slope evaluations on a curve; the weighted
average; local O(h⁵) / global O(h⁴); lesson 12: every stage needs the full
trial state.

> To move the model forward in time, we use the classic fourth-order
> Runge-Kutta method. Instead of trusting the slope at the start of a step,
> it samples four slopes: at the start, twice at the midpoint, and at the end
> - each one using the previous estimate. A weighted average of the four is
> accurate to fourth order: halve the step, and the error drops sixteen-fold.
> One lesson from development: an early version used the current body rates
> in every stage instead of the trial state. Each stage must see the full
> trial state, or it isn't RK4 any more.

## S06CascadedPID (≈ 45 s)

**On screen:** the three nested loops; why they're nested; tilt from
acceleration, T cos θ = mg and T sin θ = m a ⇒ θ = arctan(a/g);
tilt-compensated thrust.

> The first controller is a cascade of three PID loops. The slow outer loop
> turns position error into a desired acceleration; the middle loop turns
> attitude error into body rates; the fast inner loop turns rate error into
> torque. Nesting works because each inner loop is much faster than the one
> outside it. How do we get a tilt from an acceleration? Holding altitude
> needs T cos theta equal to m g; the horizontal force left over is T sin theta
> equals m a. Dividing: theta equals arc-tangent of a over g. And since only
> part of the thrust points up, we divide by cos phi cos theta.

## S07Tracking (≈ 30 s)

**On screen:** top view of the figure-eight: reference and the PID flight;
the RMS errors.

> Here's the cascaded PID flying the seven-second, climbing figure-eight.
> It starts tilted and spinning, falls behind for the first second, then
> follows the path with an RMS error of about fifteen centimetres
> horizontally and one centimetre in altitude. The small lag is because the position loop only reacts to error
> - it doesn't know where the path is going next.

## S08LQRDerivation (≈ 50 s)

**On screen:** linearization at hover (chains of integrators); the
quadratic cost; the HJB equation; the optimal gain; the Riccati equation
boxed.

> The second controller is the linear-quadratic regulator. Near hover, the
> dynamics become linear: torque drives rates, rates drive angles, tilt drives
> horizontal acceleration. We choose a cost: the integral of x-transpose Q x,
> which penalizes error, plus u-transpose R u, which penalizes effort. Guess
> that the best achievable cost from state x is x-transpose P x. The
> Hamilton-Jacobi-Bellman equation then asks for the u that minimizes a
> quadratic - setting its gradient to zero gives u equals minus R-inverse
> B-transpose P x. Substituting back, the equation must hold for every x,
> and that leaves the algebraic Riccati equation for P.

## S09Hamiltonian (≈ 40 s)

**On screen:** the Hamiltonian matrix; the ±λ symmetry and its one-line
proof; eigenvalues of our drone's Hamiltonian in the complex plane; P = X₂X₁⁻¹.

> How do we solve it? Pair the state with a costate, and both evolve under one
> matrix: the Hamiltonian. Its eigenvalues always come in plus-minus pairs -
> J times H is symmetric, so H is similar to minus H-transpose. Here are the
> twenty-four eigenvalues for our drone: perfectly mirrored. The optimal
> motion must decay, so it lives in the stable half. Take the eigenvectors for
> those, split them into top and bottom halves, and P is X-two times X-one
> inverse. Our code does exactly this, without any library solver.

## S10Limits (≈ 40 s)

**On screen:** finding 11. LQR recovery pitch reaching 58°; the RMSE table
PID vs LQR; caveat 13; end card with the docs link.

> But LQR is only optimal for the linear model near hover. In the recovery
> demo it pitches the drone to fifty-eight degrees - far outside that region -
> and still converges, thanks to its robust stability margins. On the moving
> figure-eight, though, it tracks two to ten times worse than the PID: it was
> designed to hold a point, not follow a path. And one caveat for the whole
> project: these controllers fly on the true state. Flying on the filters'
> estimates is the next step. Thanks for watching - the full derivations are
> in the documentation.
