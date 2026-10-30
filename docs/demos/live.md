# Live demo (OpenCR)

```powershell
python -m drone_sim.live --port COM5
```

| Option | Default | Meaning |
|---|---|---|
| `--port` | (required) | Serial port, e.g. `COM5` or `/dev/ttyACM0` |
| `--baud` | `115200` | Must match the firmware |
| `--rate` | `100` | Firmware sample rate in Hz |

## Recording

The live viewer running with a real OpenCR board:

```{youtube} wJ4cVM0f4uc
:title: Live OpenCR demo
```

## What you see

- **3D board model** (left). It is rotated by the EKF attitude estimate. The red
  face and the red axis mark the board's nose (+X).
- **Virtual flight.** An IMU cannot measure absolute position, so the model
  "flies" forward along its nose at 1.2 m/s: tilt the nose up to climb, roll
  and yaw to turn. Altitude is clamped to 0-3 m. The camera follows the model.
- **Read-out** (top left): roll, pitch, yaw in degrees, the virtual position,
  and the calibration status.
- **Gyroscope and accelerometer charts** (right): the last 10 s of raw samples.

## Startup calibration

Keep the board **still and roughly level** after starting. The viewer collects
about 1.5 s of consecutive quiet samples, then:

1. averages the gyro to get its **bias**, subtracted from every later sample;
2. checks that the accelerometer reads about 1 g, which also detects the
   older firmware's 0.25 g / 4 g scale;
3. initializes roll and pitch from the direction of gravity.

Any movement restarts the window, and the status line says why. Heading starts
at zero: gravity cannot reveal which way the board points.

## Why the estimate stays stable

- While the board is **still**, the accelerometer is a clean gravity reference
  and corrects roll and pitch.
- While it **moves**, the accelerometer also measures your hand's acceleration,
  so the correction is switched off and the gyro carries the attitude.
- The magnetometer (MPU9250-based boards only) corrects yaw only when the
  board is still and the field strength and direction look undisturbed.

See {doc}`/theory/ekf` for the filter and {doc}`/findings/index` for why these
rules exist.

## Troubleshooting

`could not open COM5`
: Wrong port or the port is held by another program (close the Arduino Serial
  Monitor).

Stuck on "invalid accelerometer scale"
: The firmware's accelerometer scaling does not match; reflash the sketch in
  this repository.

Stuck on "motion detected"
: Put the board on a table during startup; vibration counts as motion.
