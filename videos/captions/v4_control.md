# Video 4 - Dynamics and control: RK4, PID and LQR (on-screen captions)

One bullet = one caption card, shown in order across the scene.

## S01Intro
- The simulated drone starts 5 m off target, tilted and spinning, and recovers in a few seconds.
- That takes three things: a physics model, an accurate integrator, and a controller.
- This video derives all three.

## S02NewtonEuler
- Translation is Newton's second law: rotated thrust plus drag, divided by mass, plus gravity.
- Rotation is Euler's equation. Angular momentum is Iω in body axes…
- …but the body axes themselves rotate, so τ = I·dω/dt + ω × (Iω).
- The cross product is the gyroscopic term: spinning about one axis while pitching causes a roll acceleration.
- For a symmetric quadcopter this gives three simple equations, exactly as in the plant model.

## S03MotorMixing
- Four motors produce everything. Each pushes along body −z with force kt·ω².
- Each also resists its own spin with a torque kb·ω².
- In an X frame each motor sits 45° off the axes, so its lever arm is a = ℓ/√2.
- Each force's torque is r × F. Collecting all four gives one mixing matrix M.
- Two motors spin clockwise, two counter-clockwise: that is what makes yaw control possible.

## S04Saturation
- M is invertible, so every request has a solution on paper. But motors cannot spin backwards, and they have a top speed.
- On the first step of the demo, the controller asks for about 10× more torque than the motors can give.
- Clipping each motor separately once turned a 12.6 N thrust request into 52 N.
- The allocator keeps the thrust and scales the torque down by the largest factor λ that keeps every motor feasible.
- Holding altitude beats turning fast.

## S05RK4
- To move the model forward in time: the classic fourth-order Runge-Kutta method.
- It samples four slopes: at the start, twice at the midpoint, and at the end, each using the previous estimate.
- Their weighted average is accurate to fourth order: halve the step, and the error drops 16×.
- Lesson learned: an early version reused the current body rates in every stage.
- Each stage must see the full trial state, or it is not RK4 any more.

## S06CascadedPID
- The first controller is a cascade of three PID loops.
- Outer loop: position error → desired acceleration. Middle: attitude error → body rates. Inner: rate error → torque.
- Nesting works because each inner loop is much faster than the one outside it.
- Tilt from acceleration: T cos θ = mg and T sin θ = ma, so θ = arctan(a/g).
- Only part of the thrust points up, so the thrust is divided by cos φ cos θ.

## S07Tracking
- The cascaded PID flying the 7 s climbing figure-eight.
- It starts tilted and spinning, falls behind for the first second, then catches up.
- RMS error: about 15 cm horizontally and 1 cm in altitude. The lag comes from reacting only to error.

## S08LQRDerivation
- The second controller is the linear-quadratic regulator (LQR).
- Near hover the dynamics are linear: torque → rates → angles → horizontal acceleration.
- The cost J adds up state error xᵀQx and control effort uᵀRu over time.
- Guess the best achievable cost is xᵀPx. Minimizing the HJB equation over u gives u* = −R⁻¹BᵀPx.
- Substituting back, the equation must hold for every x. What remains is the algebraic Riccati equation for P.

## S09Hamiltonian
- To solve it, pair the state with a costate. Both evolve under one matrix: the Hamiltonian H.
- Its eigenvalues always come in ± pairs, because JH is symmetric, so H is similar to −Hᵀ.
- The 24 eigenvalues of this drone's Hamiltonian: perfectly mirrored.
- The optimal motion must decay, so it lives in the stable half: P = X₂X₁⁻¹.
- The code does exactly this, with no library solver.

## S10Limits
- LQR is optimal only for the linear model near hover.
- In the recovery demo it pitches the drone to 58°, far outside that region, and still converges.
- On the moving figure-eight it tracks 2-10× worse than the PID: it was designed to hold a point, not follow a path.
- Caveat: both controllers fly on the true state. Flying on the filters' estimates is the next step.
- Full derivations are in the documentation.
