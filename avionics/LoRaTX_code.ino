#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <MPU6500_WE.h>
#include <SPI.h>
#include <LoRa.h>
#include <ESP32Servo.h>

// ---------------- Servo Definations ----------------
Servo myservo;
#define SERVO_PIN 25          // attach servo to GPIO 25 (you can change this)
const int servo_on = 180;     // servo ON position
const int servo_off = 0;      // servo OFF position
int servo_status = 0;         // 0 = off, 1 = on 

// ---------------- LoRa ----------------
#define LORA_SS   5
#define LORA_RST  27
#define LORA_DIO0 26
#define LORA_FREQ 433E6

// ---------------- GPS ----------------
TinyGPSPlus gps;
#define gpsSerial Serial2

// ---------------- BMP280 ----------------
#define SDA_PIN 21
#define SCL_PIN 22
Adafruit_BMP280 bmp;
float seaLevelPressure; // calibrated pressure at start

// ---------------- MPU6500 ----------------
#define MPU6500_ADDR 0x68
MPU6500_WE myMPU6500(MPU6500_ADDR);

float pitch = 0, roll = 0, yaw = 0;
float vx = 0, vy = 0, vz = 0;
float prev_ax = 0, prev_ay = 0, prev_az = 0;
unsigned long lastTime;
const float G = 9.81;
const float ACC_MAG_TOL_G = 0.02;  // tighter stationary detection
const float GYR_TOL_DEG = 1.5;
int stationary_counter = 0;
const int STATIONARY_COUNT_TO_RESET = 5; // more strict

unsigned long lastGPSPrint = 0;
const unsigned long GPS_INTERVAL = 1000; // 1 Hz
unsigned long lastBMPPrint = 0;
const unsigned long BMP_INTERVAL = 1000; // 1 Hz

// Simple low-pass filter for acceleration
float alpha_acc = 0.85;
float ax_filt = 0, ay_filt = 0, az_filt = 0;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  SPI.begin(18, 19, 23, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
    while (1);
  }
  Serial.println("LoRa init OK, transmitting...");

  Serial.println("\n=== Initializing BMP280 ===");
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 not found!");
    while (1);
  }
  seaLevelPressure = bmp.readPressure() / 100.0F; // relative altitude
  Serial.print("BMP280 calibrated. Base pressure: ");
  Serial.print(seaLevelPressure);
  Serial.println(" hPa");

  // --- Servo setup ---
  myservo.attach(SERVO_PIN);
  myservo.write(servo_off); // start OFF


  Serial.println("\n=== Initializing MPU6500 ===");
  if (!myMPU6500.init()) {
    Serial.println("MPU6500 not found!");
    while (1);
  }
  myMPU6500.autoOffsets();
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_8G);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_500);
  lastTime = millis();

  Serial.println("\n=== Initialization Complete ===");
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  lastTime = now;

  // --------------- MPU6500 ---------------
  xyzFloat acc = myMPU6500.getGValues();
  xyzFloat gyr = myMPU6500.getGyrValues();

  // --- pitch/roll/yaw ---
  float accPitch = atan2(acc.y, sqrt(acc.x*acc.x + acc.z*acc.z)) * 180.0 / PI;
  float accRoll  = atan2(-acc.x, acc.z) * 180.0 / PI;
  const float alpha = 0.96;
  pitch = alpha * (pitch + gyr.x*dt) + (1-alpha)*accPitch;
  roll  = alpha * (roll + gyr.y*dt) + (1-alpha)*accRoll;
  yaw  += gyr.z * dt;

  // --- world-frame acceleration ---
  float pr = pitch * PI/180.0, rr = roll*PI/180.0, yr = yaw*PI/180.0;
  float cr=cos(rr), sr=sin(rr), cp=cos(pr), sp=sin(pr), cy=cos(yr), sy=sin(yr);
  float R11=cy*cp, R12=cy*sp*sr - sy*cr, R13=cy*sp*cr + sy*sr;
  float R21=sy*cp, R22=sy*sp*sr + cy*cr, R23=sy*sp*cr - cy*sr;
  float R31=-sp, R32=cp*sr, R33=cp*cr;
  float ax_world = (R11*acc.x + R12*acc.y + R13*acc.z)*G;
  float ay_world = (R21*acc.x + R22*acc.y + R23*acc.z)*G;
  float az_world = (R31*acc.x + R32*acc.y + R33*acc.z)*G - G;

  // --- low-pass filter to reduce noise ---
  ax_filt = alpha_acc*ax_filt + (1-alpha_acc)*ax_world;
  ay_filt = alpha_acc*ay_filt + (1-alpha_acc)*ay_world;
  az_filt = alpha_acc*az_filt + (1-alpha_acc)*az_world;

  // --- stationary detection ---
  float acc_mag_g = sqrt(acc.x*acc.x + acc.y*acc.y + acc.z*acc.z);
  bool stationary = (fabs(acc_mag_g - 1.0) < ACC_MAG_TOL_G) &&
                    (fabs(gyr.x)<GYR_TOL_DEG && fabs(gyr.y)<GYR_TOL_DEG && fabs(gyr.z)<GYR_TOL_DEG);
  if (stationary) stationary_counter++; else stationary_counter=0;

  if (stationary_counter >= STATIONARY_COUNT_TO_RESET) vx=vy=vz=0;
  else {
    vx += 0.5*(prev_ax + ax_filt)*dt;
    vy += 0.5*(prev_ay + ay_filt)*dt;
    vz += 0.5*(prev_az + az_filt)*dt;
  }
  prev_ax=ax_filt; prev_ay=ay_filt; prev_az=az_filt;

  // --- Print MPU6500 data ---
  Serial.print("MPU | P:"); Serial.print(pitch,2);
  Serial.print(" R:"); Serial.print(roll,2);
  Serial.print(" Y:"); Serial.print(yaw,2);
  Serial.print(" | ax:"); Serial.print(ax_filt,2);
  Serial.print(" ay:"); Serial.print(ay_filt,2);
  Serial.print(" az:"); Serial.print(az_filt,2);
  Serial.print(" | Vx:"); Serial.print(vx,3);
  Serial.print(" Vy:"); Serial.print(vy,3);
  Serial.print(" Vz:"); Serial.println(vz,3);

  // --------------- GPS (1 Hz) ---------------
  while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());
  if (now - lastGPSPrint >= GPS_INTERVAL) {
    lastGPSPrint = now;
    Serial.println(F("\n--- GPS DATA ---"));
    if (gps.location.isValid()) {
      Serial.print("Lat: "); Serial.print(gps.location.lat(),6);
      Serial.print(" Lon: "); Serial.print(gps.location.lng(),6);
      Serial.println();
      Serial.print("Satellites: "); Serial.println(gps.satellites.value());
      Serial.print("Altitude: "); Serial.print(gps.altitude.meters()); Serial.println(" m");
      Serial.print("Speed: "); Serial.print(gps.speed.kmph()); Serial.println(" km/h");
      Serial.print("Course: "); Serial.print(gps.course.deg()); Serial.println(" deg");
      if (gps.date.isValid()) Serial.printf("Date: %02d/%02d/%04d\n", gps.date.day(), gps.date.month(), gps.date.year());
      if (gps.time.isValid()) Serial.printf("Time (UTC): %02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
    } else Serial.println("No GPS fix yet.");
  }

  // --------------- BMP280 (1 Hz) ---------------
  float temperature, pressure, altitude;
  if (now - lastBMPPrint >= BMP_INTERVAL) {
    lastBMPPrint = now;
    temperature = bmp.readTemperature();
    pressure = bmp.readPressure()/100.0F;
    altitude = bmp.readAltitude(seaLevelPressure); // relative
    Serial.print("BMP | Temp: "); Serial.print(temperature);
    Serial.print(" °C, Pressure: "); Serial.print(pressure);
    Serial.print(" hPa, Altitude: "); Serial.print(altitude); Serial.println(" m");
  }

  // --- Servo trigger logic ---
  if (altitude >= 1000 && servo_status == 0) {
    myservo.write(servo_on);      // activate servo
    servo_status = 1;             // update status
    Serial.println(">>> Servo Activated! Altitude >= 1000m <<<");
  }

  String payload = String(gps.location.lat(),6) + "," + String(gps.location.lng(),6) + "," +
            String(altitude) + "," + String(pressure) + "," + String(temperature) + "," +
            String(pitch,2) + "," + String(roll,2) + "," + String(yaw,2) + "," +
            String(ax_filt,2) + "," + String(ay_filt,2) + "," + String(az_filt,2) + "," +
            String(vx,3) + "," + String(vy,3) + "," + String(vz,3) + "," +
            String(servo_status); 

  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();

  delay(1000); 
}
