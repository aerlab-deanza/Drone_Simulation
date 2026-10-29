# Hardware: OpenCR IMU stream

The live demo reads a [ROBOTIS OpenCR](https://emanual.robotis.com/docs/en/parts/controller/opencr10/)
board's on-board IMU over USB.

## Flashing the firmware

1. Install the Arduino IDE and add the OpenCR board package (*Preferences →
   Additional Boards Manager URLs*, then install **OpenCR** in the Boards
   Manager).
2. Open `firmware/opencr_imu_stream/opencr_imu_stream.ino`.
3. Select *Tools → Board → OpenCR* and the board's port, then **Upload**.
4. Optional check: the Serial Monitor at 115200 baud shows a `#` header line
   followed by CSV lines. **Close it** before starting the viewer, since only
   one program can hold the port.

## Stream format

One line per sample at 100 Hz:

```text
# OpenCR IMU CSV: time_us,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g,mx,my,mz
1234567,0.061035,-0.122070,0.000000,0.001953,-0.003906,1.000488,0,0,0
```

| Field | Unit | Converted by the reader to |
|---|---|---|
| `time_us` | µs (`micros()`, wraps after ~71 min) | seconds between samples |
| `gx, gy, gz` | deg/s | rad/s |
| `ax, ay, az` | g | m/s² (× 9.80665) |
| `mx, my, mz` | raw counts; all zero if absent | unchanged, or `None` |

Parsing: {func}`~drone_sim.sensors.opencr_serial.parse_imu_line`. Malformed
lines are counted in
{attr}`OpenCRSerialReader.bad_lines <drone_sim.sensors.opencr_serial.OpenCRSerialReader>`
and skipped.

## Accelerometer scaling

OpenCR library 1.5.3 scales the accelerometer as if it were configured for
±8 g, while both bundled IMU drivers configure ±2 g. The sketch multiplies by
`2/8` to output physical g. If you port it to another library version, check
the configured range. The viewer's startup calibration still recognizes
streams that are off by exactly ×4 or ×¼ and corrects them
({func}`~drone_sim.visualization.live_viewer.startup_accel_scale`).

## Magnetometer

MPU9250-based OpenCR boards report a magnetometer; ICM-20648 boards send
zeros, which the reader treats as "no magnetometer". Without one, yaw is
gyro-only and drifts slowly ({doc}`/theory/imu-noise`).

## Full sketch

```{literalinclude} ../firmware/opencr_imu_stream/opencr_imu_stream.ino
:language: cpp
```
