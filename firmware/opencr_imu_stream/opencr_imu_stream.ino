#include <IMU.h>

// OpenCR's bundled IMU library supports the board's configured IMU. On
// MPU9250 boards it supplies magnetometer data; ICM-20648 boards send zeros
// for the magnetometer fields, which the Python reader treats as unavailable.
cIMU imu;
const uint32_t SAMPLE_RATE_HZ = 100;

void setup() {
  Serial.begin(115200);
  const uint32_t wait_start = millis();
  while (!Serial && (millis() - wait_start < 3000)) {
    // Give the USB serial connection a moment to enumerate.
  }

  imu.begin(SAMPLE_RATE_HZ);
  if (!imu.bConnected) {
    Serial.println("# ERROR: OpenCR IMU was not detected");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("# OpenCR IMU CSV: time_us,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g,mx,my,mz");
}

void loop() {
  if (imu.update() == 0) {
    return;
  }
  
  // OpenCR 1.5.3 sets cIMU::aRes to 8/32768, while both bundled IMU drivers
  // configure the accelerometer for +/-2 g. Convert that library scaling to
  // physical g here. Python's CSV contract is gx/gy/gz in deg/s and ax/ay/az
  // in g. Keep the factor explicit: check the selected library range before
  // porting this sketch to a different OpenCR package version.
  const float OPENCR_LIBRARY_ACCEL_RANGE_G = 8.0f;
  const float SENSOR_ACCEL_RANGE_G = 2.0f;
  const float accel_library_to_g = SENSOR_ACCEL_RANGE_G / OPENCR_LIBRARY_ACCEL_RANGE_G;
  Serial.print(micros());
  Serial.print(','); Serial.print(imu.gx, 6);
  Serial.print(','); Serial.print(imu.gy, 6);
  Serial.print(','); Serial.print(imu.gz, 6);
  Serial.print(','); Serial.print(imu.ax * accel_library_to_g, 6);
  Serial.print(','); Serial.print(imu.ay * accel_library_to_g, 6);
  Serial.print(','); Serial.print(imu.az * accel_library_to_g, 6);
  Serial.print(','); Serial.print(imu.mx, 6);
  Serial.print(','); Serial.print(imu.my, 6);
  Serial.print(','); Serial.println(imu.mz, 6);
}
