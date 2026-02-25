#include <Wire.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;

// Rotation matrix
float R[3][3];

// Time tracking
unsigned long lastTime;

// ---------- Initialize Identity ----------
void initIdentity() {
  R[0][0] = 1; R[0][1] = 0; R[0][2] = 0;
  R[1][0] = 0; R[1][1] = 1; R[1][2] = 0;
  R[2][0] = 0; R[2][1] = 0; R[2][2] = 1;
}

// ---------- Matrix Multiply ----------
void multiplyMatrix(float A[3][3], float B[3][3], float result[3][3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      result[i][j] = 0;
      for (int k = 0; k < 3; k++) {
        result[i][j] += A[i][k] * B[k][j];
      }
    }
  }
}

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

  Wire.begin(21, 22);  // SDA=21, SCL=22

  if (!imu.begin()) {
    while (1);
  }

  initIdentity();
  lastTime = micros();
}

void loop() {

  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) * 1e-6;
  lastTime = currentTime;

  imu.readGyro();

  float gx = imu.calcGyro(imu.gx) * PI / 180.0;
  float gy = imu.calcGyro(imu.gy) * PI / 180.0;
  float gz = imu.calcGyro(imu.gz) * PI / 180.0;

  float omega_norm = sqrt(gx*gx + gy*gy + gz*gz);

  float R_delta[3][3];

  if (omega_norm > 1e-6) {

    float theta = omega_norm * dt;

    float kx = gx / omega_norm;
    float ky = gy / omega_norm;
    float kz = gz / omega_norm;

    float c = cos(theta);
    float s = sin(theta);
    float v = 1 - c;

    R_delta[0][0] = kx*kx*v + c;
    R_delta[0][1] = kx*ky*v - kz*s;
    R_delta[0][2] = kx*kz*v + ky*s;

    R_delta[1][0] = ky*kx*v + kz*s;
    R_delta[1][1] = ky*ky*v + c;
    R_delta[1][2] = ky*kz*v - kx*s;

    R_delta[2][0] = kz*kx*v - ky*s;
    R_delta[2][1] = kz*ky*v + kx*s;
    R_delta[2][2] = kz*kz*v + c;

  } else {

    R_delta[0][0] = 1; R_delta[0][1] = 0; R_delta[0][2] = 0;
    R_delta[1][0] = 0; R_delta[1][1] = 1; R_delta[1][2] = 0;
    R_delta[2][0] = 0; R_delta[2][1] = 0; R_delta[2][2] = 1;
  }

  float R_new[3][3];
  multiplyMatrix(R, R_delta, R_new);

  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      R[i][j] = R_new[i][j];

  float qx, qy, qz, qw;
  rotationToQuaternion(R, qx, qy, qz, qw);

  Serial.print(qx); Serial.print(",");
  Serial.print(qy); Serial.print(",");
  Serial.print(qz); Serial.print(",");
  Serial.println(qw);

  delay(10);   // ~100 Hz
}
