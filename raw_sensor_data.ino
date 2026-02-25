#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// Gyro bias (rad/s)
float gyro_bias_x = 0;
float gyro_bias_y = 0;
float gyro_bias_z = 0;

// =======================
// Gyro Calibration
// =======================
void calibrateGyro() {

  Serial.println("=================================");
  Serial.println("Keep board completely still...");
  Serial.println("Calibrating gyro in 3 seconds...");
  Serial.println("=================================");

  delay(3000);

  const int N = 5000;

  for (int i = 0; i < N; i++) {

    sensors_event_t accel, gyro, mag, temp;
    lsm.getEvent(&accel, &gyro, &mag, &temp);

    // Convert to rad/s FIRST
    float gx = gyro.gyro.x * DEG_TO_RAD;
    float gy = gyro.gyro.y * DEG_TO_RAD;
    float gz = gyro.gyro.z * DEG_TO_RAD;

    gyro_bias_x += gx;
    gyro_bias_y += gy;
    gyro_bias_z += gz;

    delay(2);
  }

  gyro_bias_x /= N;
  gyro_bias_y /= N;
  gyro_bias_z /= N;

  Serial.println("Gyro bias (rad/s):");
  Serial.print(gyro_bias_x); Serial.print(", ");
  Serial.print(gyro_bias_y); Serial.print(", ");
  Serial.println(gyro_bias_z);

  Serial.println("Calibration complete.");
}

// =======================
// Setup
// =======================
void setup() {

  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);

  if (!lsm.begin()) {
    Serial.println("LSM9DS1 not detected!");
    while (1);
  }

  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  Serial.println("Warming up gyro...");
  delay(5000);   // 🔥 IMPORTANT

  calibrateGyro();
}

// =======================
// Main Loop
// =======================
void loop() {

  sensors_event_t accel, gyro, mag, temp;
  lsm.getEvent(&accel, &gyro, &mag, &temp);

  // Convert to rad/s and subtract bias
  // Raw converted values
float gxr = gyro.gyro.x * DEG_TO_RAD - gyro_bias_x;
float gyr = gyro.gyro.y * DEG_TO_RAD - gyro_bias_y;
float gzr = gyro.gyro.z * DEG_TO_RAD - gyro_bias_z;

// Remap axes
float gx = gyr;   // physical X rotation
float gy = gzr;   // physical Y rotation
float gz = gxr;   // physical Z rotation

  Serial.print("gx: ");
  Serial.print(gx, 4);
  Serial.print(" | gy: ");
  Serial.print(gy, 4);
  Serial.print(" | gz: ");
  Serial.println(gz, 4);

  delay(100);
}
