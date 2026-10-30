# Video 2 - IMU noise and drift (on-screen captions)

One bullet = one caption card, shown in order across the scene.

## S01Intro
- This sensor is sitting perfectly still. Adding up its gyro readings gives an angle…
- …and that angle slowly wanders away from zero.
- Why does that happen, how fast, and what can be done about it?

## S02ErrorModel
- Each reading is modelled as four parts.
- The true value, plus a constant bias that calibration can remove…
- …plus a bias instability that wanders slowly…
- …plus white noise, fresh and independent at every sample.
- The numbers come from measurements of real MPU6050 chips (Gonzalez & Dabove, 2019).

## S03WhyGaussian
- Why Gaussian noise? Sensor noise is the sum of many small, independent effects.
- The central limit theorem: such sums tend toward a bell curve, whatever each effect looks like.
- Here, averages of a very lopsided distribution become Gaussian as more samples are averaged.

## S04DensityToSamples
- Datasheets give white noise as a density N, in units per √Hz.
- Sampled at rate fs, each sample has standard deviation σ = N·√fs.
- 4× the sample rate gives 2× noisier samples, but the same noise per second.
- MPU6050 gyro at 100 Hz: about 6.6 × 10⁻⁴ rad/s per sample.

## S05RandomWalk
- Now integrate. Over time t there are t·fs samples, each multiplied by the time step.
- The variances add: n·σ²·Δt² = (t·fs)(N²·fs)/fs² = N²·t. The sample rate cancels.
- So the angle error grows like N·√t: a random walk.
- 200 simulated runs spread out exactly along that curve.
- After 7 s this gyro has drifted only about a hundredth of a degree.

## S06GaussMarkov
- The bias instability is a Gauss-Markov process: keep a fraction φ of the bias, add a little fresh noise.
- With φ = e^(−Δt/τ), the bias forgets itself over a correlation time τ.
- In steady state the variance stops changing: σ² = φ²σ² + q².
- Choosing q = B·√(1 − φ²) makes the spread exactly B.

## S07TwoRegimes
- A constant bias b integrates to b·t: linear, not square-root.
- Early on the random walk dominates; later the bias takes over.
- They are equal at t = N²/B², about 17 s for this gyro.
- The 200 simulated runs follow the combined curve √(N²t + (Bt)²) almost perfectly.

## S08IntegrationError
- A surprise from the simulator: in the 7 s flight demo, the attitude error peaks at about 0.16°.
- Remove all sensor noise, with a perfect IMU, and the peak is the same.
- The culprit is the filter's integration step q + (dq/dt)·Δt, whose error grows with the rotation rate.
- At a 4× faster sample rate the error shrinks 4×: the signature of a first-order method.
- Lesson: before blaming the sensor, check the integrator.

## S09PositionDrift
- Position is far worse, because the accelerometer is integrated twice.
- White noise grows like t^(3/2), a bias like t².
- The biggest culprit is usually tilt: a 0.1° attitude error leaks 0.017 m/s² of gravity sideways…
- …which becomes almost a metre of position error after 10 s.

## S10Outro
- The gyro is precise over short times but drifts. Gravity never drifts but is noisy.
- Combine them and the drift disappears: that is sensor fusion.
- Next: the Madgwick filter and the extended Kalman filter.
