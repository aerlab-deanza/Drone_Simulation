# Video 2 - IMU noise and drift

Target length: about 5.5 minutes. One audio file per scene, named by scene
ID (e.g. `S01Intro.wav`), in `videos/audio/v2_imu_noise/`.

---

## S01Intro (≈ 20 s)

**On screen:** a motionless IMU; its integrated gyro angle slowly wanders
away from zero; title.

> This sensor is sitting perfectly still on a table. Yet if we add up its
> gyroscope readings to get an angle, the angle slowly wanders away from
> zero. Why does that happen, how fast, and what can we do about it?
> Let's build the noise model our simulator uses, and derive the drift
> exactly.

## S02ErrorModel (≈ 35 s)

**On screen:** measured = true + constant bias + bias instability + white
noise; each term highlighted; the MPU6050 parameter table.

> Every reading from each axis is modelled as four parts. The true value. A
> constant bias, a fixed offset that calibration can remove. A bias
> instability - an offset that slowly wanders. And white noise, which is
> fresh and independent at every single sample. The numbers in our simulator
> come from measurements of real MPU-6050 chips, published by Gonzalez and
> Dabove in 2019.

## S03WhyGaussian (≈ 30 s)

**On screen:** histograms of means of exponential samples, N = 1, 5, 30,
morphing into a bell curve.

> Why model the noise as Gaussian? Because sensor noise is the sum of many
> small, independent effects - thermal motion, electronics, rounding. And the
> central limit theorem says that sums of independent effects tend toward a
> bell curve, almost regardless of what each effect looks like on its own.
> Here, averages of a very lopsided distribution become Gaussian as we
> average more of them.

## S04DensityToSamples (≈ 30 s)

**On screen:** σ = N√fs; sample streams at 100 Hz and 400 Hz; the
MPU6050 value.

> Datasheets give white noise as a density, N, in units per root hertz.
> When we sample at a rate f-s, each sample has a standard deviation of N
> times the square root of f-s. Sampling four times faster makes each sample
> twice as noisy - but, as we'll see next, the noise per second stays the
> same. For the MPU-6050's gyro at a hundred hertz, that's about
> six-point-six times ten to the minus four radians per second.

## S05RandomWalk (≈ 45 s)

**On screen:** derivation Var = n σ² Δt² = N² t; forty random walks
spreading; the √t envelope; 0.010° at 7 s.

> Now integrate. Over a time t there are t times f-s samples, each scaled by
> the time step. The variances add up: the number of samples, times sigma
> squared, times delta-t squared. The sample rate cancels, and we're left
> with N squared times t. So the standard deviation of the angle error grows
> like N times the square root of t - a random walk. Run it two hundred times
> and the spread follows that curve exactly. For our sensor, after seven
> seconds, that's just a hundredth of a degree.

## S06GaussMarkov (≈ 35 s)

**On screen:** b[k+1] = φ b[k] + q w[k]; proof that the stationary
variance is B²; a sample bias sequence.

> The bias instability is modelled as a Gauss-Markov process: each step keeps
> a fraction phi of the previous bias and adds a little fresh noise. With
> phi equal to e to the minus delta-t over tau, the bias forgets itself over a
> correlation time tau. What size of fresh noise keeps its spread at exactly
> B? In steady state the variance doesn't change, so sigma squared equals
> phi squared sigma squared plus q squared. Choose q equal to B times the
> square root of one minus phi squared, and the spread is exactly B.

## S07TwoRegimes (≈ 35 s)

**On screen:** measured spread vs N√t vs √(N²t + (Bt)²); crossover at
about 17 s.

> A bias integrates very differently from noise. A constant offset b produces
> an angle error b times t - linear, not square-root. Put the two together:
> early on, the random walk dominates; later, the bias takes over. The two
> are equal when t equals N squared over B squared - about seventeen seconds
> for this gyro. Our two hundred simulated runs follow the combined curve
> almost perfectly.

## S08IntegrationError (≈ 45 s)

**On screen:** finding 4. The demo's attitude error with a noisy IMU and with
a noise-free IMU - nearly identical; then the 2 kHz curve, four times
smaller; the table.

> Now a surprise from our own simulator. In the seven-second flight demo, the
> attitude error peaks at about point one six degrees. Remove all sensor noise
> - a perfect IMU - and the peak is… exactly the same. The noise isn't the
> main culprit. The filter advances its quaternion with one simple step, q
> plus q-dot times delta-t, and that step makes an error that grows with the
> rotation rate. Run four times faster, and the error shrinks four times - the
> signature of a first-order method. Lesson: before blaming the sensor, check
> the integrator.

## S09PositionDrift (≈ 35 s)

**On screen:** the double-integration table; a 0.1° tilt error leaking
gravity; position error growing to about 0.9 m in 10 s.

> Position is much worse, because the accelerometer is integrated twice.
> White noise now grows like t to the three-halves, and any bias like t
> squared. And the biggest culprit is usually tilt: if the estimated attitude
> is off by just a tenth of a degree, a sliver of gravity - about
> zero-point-zero-one-seven metres per second squared - leaks into the
> horizontal axes. After ten seconds, that's almost a metre of error.

## S10Outro (≈ 20 s)

**On screen:** a biased gyro: integration drifts by 17°, fusion stays flat;
"Next: sensor fusion".

> So the gyroscope is precise over short times but drifts over long ones,
> while gravity and the magnetic field don't drift but are noisy. Combine
> them, and the drift disappears - that's sensor fusion, the subject of the
> next video.
