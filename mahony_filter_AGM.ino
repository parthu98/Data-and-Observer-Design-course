#include <Wire.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;

// Rotation matrix
float R[3][3];

// Bias estimate
float bx = 0, by = 0, bz = 0;

// Gains (tune if needed)
float kp = 0.5   ;
float ki = 0.002;

unsigned long lastTime;

// ---------- Identity ----------
void initIdentity() {
  R[0][0]=1; R[0][1]=0; R[0][2]=0;
  R[1][0]=0; R[1][1]=1; R[1][2]=0;
  R[2][0]=0; R[2][1]=0; R[2][2]=1;
}

// ---------- Matrix Multiply ----------
void multiplyMatrix(float A[3][3], float B[3][3], float result[3][3]) {
  for (int i=0;i<3;i++){
    for (int j=0;j<3;j++){
      result[i][j]=0;
      for (int k=0;k<3;k++)
        result[i][j]+=A[i][k]*B[k][j];
    }
  }
}

// ---------- Quaternion ----------
void rotationToQuaternion(float R[3][3], float &qx, float &qy, float &qz, float &qw) {

  float trace = R[0][0] + R[1][1] + R[2][2];

  if (trace > 0) {
    float s = sqrt(trace + 1.0) * 2;
    qw = 0.25 * s;
    qx = (R[2][1] - R[1][2]) / s;
    qy = (R[0][2] - R[2][0]) / s;
    qz = (R[1][0] - R[0][1]) / s;
  } else {
    qw = 1; qx=qy=qz=0;
  }
}

void setup() {

  Serial.begin(115200);
  delay(1000);

  Wire.begin(21,22);

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
  imu.readAccel();
  imu.readMag();

  // ---- Gyro (rad/s)
  float gx = imu.calcGyro(imu.gx) * PI/180.0;
  float gy = imu.calcGyro(imu.gy) * PI/180.0;
  float gz = imu.calcGyro(imu.gz) * PI/180.0;

  // ---- Normalize accel
  float ax = imu.calcAccel(imu.ax);
  float ay = imu.calcAccel(imu.ay);
  float az = imu.calcAccel(imu.az);
  float norm_a = sqrt(ax*ax + ay*ay + az*az);
  if (norm_a == 0) return;
  ax/=norm_a; ay/=norm_a; az/=norm_a;

  // ---- Normalize mag
  float mx = imu.calcMag(imu.mx);
  float my = imu.calcMag(imu.my);
  float mz = imu.calcMag(imu.mz);
  float norm_m = sqrt(mx*mx + my*my + mz*mz);
  if (norm_m == 0) return;
  mx/=norm_m; my/=norm_m; mz/=norm_m;

  // ---- Estimated gravity (R^T * g_I)
  float vx = R[2][0];
  float vy = R[2][1];
  float vz = R[2][2];

  // ---- Estimated magnetic field (R^T * m_I)
  float wx = R[0][0];
  float wy = R[0][1];
  float wz = R[0][2];

  // ---- Error from accel
  float ex_acc = ay*vz - az*vy;
  float ey_acc = az*vx - ax*vz;
  float ez_acc = ax*vy - ay*vx;

  // ---- Error from mag
  float ex_mag = my*wz - mz*wy;
  float ey_mag = mz*wx - mx*wz;
  float ez_mag = mx*wy - my*wx;

  // ---- Total error
  float ex = ex_acc + ex_mag;
  float ey = ey_acc + ey_mag;
  float ez = ez_acc + ez_mag;

  // ---- Bias update
  bx += -ki * ex * dt;
  by += -ki * ey * dt;
  bz += -ki * ez * dt;

  // ---- Corrected gyro
  gx = gx - bx + kp * ex;
  gy = gy - by + kp * ey;
  gz = gz - bz + kp * ez;

  // ---- Rodrigues integration
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

    R_delta[0][0]=1;R_delta[0][1]=0;R_delta[0][2]=0;
    R_delta[1][0]=0;R_delta[1][1]=1;R_delta[1][2]=0;
    R_delta[2][0]=0;R_delta[2][1]=0;R_delta[2][2]=1;
  }

  float R_new[3][3];
  multiplyMatrix(R,R_delta,R_new);

  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      R[i][j]=R_new[i][j];

  // ---- Output quaternion
  float qx,qy,qz,qw;
  rotationToQuaternion(R,qx,qy,qz,qw);

  Serial.print(qx);Serial.print(",");
  Serial.print(qy);Serial.print(",");
  Serial.print(qz);Serial.print(",");
  Serial.println(qw);

  delay(10);
}
