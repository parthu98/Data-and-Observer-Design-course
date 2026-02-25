#include <Wire.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;

// ---------- Rotation → Quaternion ----------
void rotationToQuaternion(float R[3][3], float &qx, float &qy, float &qz, float &qw) {

  float trace = R[0][0] + R[1][1] + R[2][2];

  if (trace > 0) {
    float s = sqrt(trace + 1.0) * 2;
    qw = 0.25 * s;
    qx = (R[2][1] - R[1][2]) / s;
    qy = (R[0][2] - R[2][0]) / s;
    qz = (R[1][0] - R[0][1]) / s;
  } else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) {
    float s = sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2;
    qw = (R[2][1] - R[1][2]) / s;
    qx = 0.25 * s;
    qy = (R[0][1] + R[1][0]) / s;
    qz = (R[0][2] + R[2][0]) / s;
  } else if (R[1][1] > R[2][2]) {
    float s = sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2;
    qw = (R[0][2] - R[2][0]) / s;
    qx = (R[0][1] + R[1][0]) / s;
    qy = 0.25 * s;
    qz = (R[1][2] + R[2][1]) / s;
  } else {
    float s = sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2;
    qw = (R[1][0] - R[0][1]) / s;
    qx = (R[0][2] + R[2][0]) / s;
    qy = (R[1][2] + R[2][1]) / s;
    qz = 0.25 * s;
  }
}

void setup() {

  Serial.begin(115200);
  delay(1000);

  // I2C pins for ESP32
  Wire.begin(21, 22);

  if (!imu.begin()) {
    while (1);
  }
}

void loop() {

  // Read accelerometer and magnetometer
  imu.readAccel();
  imu.readMag();

  float ax = imu.calcAccel(imu.ax);
  float ay = imu.calcAccel(imu.ay);
  float az = imu.calcAccel(imu.az);

  float mx = imu.calcMag(imu.mx);
  float my = imu.calcMag(imu.my);
  float mz = imu.calcMag(imu.mz);

  // -------------------------
  // Normalize accelerometer
  // -------------------------
  float norm_a = sqrt(ax*ax + ay*ay + az*az);
  if (norm_a == 0) return;

  ax /= norm_a;
  ay /= norm_a;
  az /= norm_a;

  // -------------------------
  // TRIAD Step 1: t1 = gravity direction
  // -------------------------
  float t1x = ax;
  float t1y = ay;
  float t1z = az;

  // -------------------------
  // TRIAD Step 2: t2 = t1 × m
  // -------------------------
  float t2x = t1y*mz - t1z*my;
  float t2y = t1z*mx - t1x*mz;
  float t2z = t1x*my - t1y*mx;

  float norm_t2 = sqrt(t2x*t2x + t2y*t2y + t2z*t2z);
  if (norm_t2 == 0) return;

  t2x /= norm_t2;
  t2y /= norm_t2;
  t2z /= norm_t2;

  // -------------------------
  // TRIAD Step 3: t3 = t1 × t2
  // -------------------------
  float t3x = t1y*t2z - t1z*t2y;
  float t3y = t1z*t2x - t1x*t2z;
  float t3z = t1x*t2y - t1y*t2x;

  // -------------------------
  // Build Rotation Matrix
  // -------------------------
  float R[3][3] = {
    {t1x, t2x, t3x},
    {t1y, t2y, t3y},
    {t1z, t2z, t3z}
  };

  // Convert to quaternion
  float qx, qy, qz, qw;
  rotationToQuaternion(R, qx, qy, qz, qw);

  // Send quaternion
  Serial.print(qx); Serial.print(",");
  Serial.print(qy); Serial.print(",");
  Serial.print(qz); Serial.print(",");
  Serial.println(qw);

  delay(20);  // ~50 Hz
}
