#include <Wire.h>
#include <MPU6500_WE.h>

// ---------------- MPU6500 ----------------
#define MPU6500_ADDR 0x68
MPU6500_WE myMPU6500(MPU6500_ADDR);

// ESP32 I2C pins
#define SDA_PIN 21
#define SCL_PIN 22

float pitch = 0, roll = 0, yaw = 0;
float vx = 0, vy = 0, vz = 0;
float prev_ax = 0, prev_ay = 0, prev_az = 0;

unsigned long lastTime;
const float G = 9.81; // gravity in m/s²

// Stationary detection thresholds
const float ACC_MAG_TOL_G = 0.03; // tolerance around 1 g
const float GYR_TOL_DEG = 2.0;    // deg/s tolerance
int stationary_counter = 0;
const int STATIONARY_COUNT_TO_RESET = 3; // consecutive samples

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("\n=== ESP32 MPU6500 Orientation + Velocity Test ===");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!myMPU6500.init()) {
    Serial.println("MPU6500 not found!");
    while (1);
  }

  myMPU6500.autoOffsets();
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_8G);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_500);

  lastTime = millis();
}

void loop() {
  // --- Read MPU6500 ---
  xyzFloat acc = myMPU6500.getGValues(); // in g
  xyzFloat gyr = myMPU6500.getGyrValues(); // in deg/s

  // --- Time step ---
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  lastTime = now;

  // --- Calculate pitch and roll from accelerometer ---
  float accPitch = atan2(acc.y, sqrt(acc.x * acc.x + acc.z * acc.z)) * 180.0 / PI;
  float accRoll  = atan2(-acc.x, acc.z) * 180.0 / PI;

  // --- Complementary filter for smoothing with gyro ---
  const float alpha = 0.98;
  pitch = alpha * (pitch + gyr.x * dt) + (1 - alpha) * accPitch;
  roll  = alpha * (roll  + gyr.y * dt) + (1 - alpha) * accRoll;
  yaw  += gyr.z * dt;

  // --- Convert accelerometer to world frame ---
  float pr = pitch * PI / 180.0;
  float rr = roll  * PI / 180.0;
  float yr = yaw   * PI / 180.0;

  float cr = cos(rr), sr = sin(rr);
  float cp = cos(pr), sp = sin(pr);
  float cy = cos(yr), sy = sin(yr);

  // rotation matrix (body -> world)
  float R11 = cy * cp;
  float R12 = cy * sp * sr - sy * cr;
  float R13 = cy * sp * cr + sy * sr;
  float R21 = sy * cp;
  float R22 = sy * sp * sr + cy * cr;
  float R23 = sy * sp * cr - cy * sr;
  float R31 = -sp;
  float R32 = cp * sr;
  float R33 = cp * cr;

  float ax_world = (R11 * acc.x + R12 * acc.y + R13 * acc.z) * G;
  float ay_world = (R21 * acc.x + R22 * acc.y + R23 * acc.z) * G;
  float az_world = (R31 * acc.x + R32 * acc.y + R33 * acc.z) * G - G; // remove gravity

  // --- Stationary detection ---
  float acc_mag_g = sqrt(acc.x*acc.x + acc.y*acc.y + acc.z*acc.z);
  bool stationary = (fabs(acc_mag_g - 1.0) < ACC_MAG_TOL_G) &&
                    (fabs(gyr.x) < GYR_TOL_DEG) && (fabs(gyr.y) < GYR_TOL_DEG) && (fabs(gyr.z) < GYR_TOL_DEG);

  if (stationary) {
    stationary_counter++;
  } else {
    stationary_counter = 0;
  }

  // --- Reset velocity if stationary ---
  if (stationary_counter >= STATIONARY_COUNT_TO_RESET) {
    vx = 0; vy = 0; vz = 0;
  } else {
    // integrate acceleration -> velocity using trapezoidal rule
    vx += 0.5 * (prev_ax + ax_world) * dt;
    vy += 0.5 * (prev_ay + ay_world) * dt;
    vz += 0.5 * (prev_az + az_world) * dt;
  }

  prev_ax = ax_world;
  prev_ay = ay_world;
  prev_az = az_world;

  // --- Print orientation, acceleration, velocity ---
  Serial.print("P:"); Serial.print(pitch,2);
  Serial.print(" R:"); Serial.print(roll,2);
  Serial.print(" Y:"); Serial.print(yaw,2);

  Serial.print(" | ax:"); Serial.print(ax_world,2);
  Serial.print(" ay:"); Serial.print(ay_world,2);
  Serial.print(" az:"); Serial.print(az_world,2);

  Serial.print(" | Vx:"); Serial.print(vx,3);
  Serial.print(" Vy:"); Serial.print(vy,3);
  Serial.print(" Vz:"); Serial.print(vz,3);

  Serial.println();

  delay(50); // 20 Hz
}
