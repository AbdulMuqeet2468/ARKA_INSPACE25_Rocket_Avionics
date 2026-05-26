//working, calculate relative altitude.
//temp readinigs are much better

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_BMP280 bmp;

float seaLevelPressure; // calibrated pressure at start

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bmp.begin(0x76)) { // try 0x77 if 0x76 doesn't work
    Serial.println("Could not find BMP280 sensor!");
    while (1);
  }

  // Calibrate sea-level pressure at startup
  seaLevelPressure = bmp.readPressure() / 100.0F; // in hPa
  Serial.print("Calibration done. Base pressure: ");
  Serial.print(seaLevelPressure);
  Serial.println(" hPa");
}

void loop() {
  float temperature = bmp.readTemperature();        // °C
  float pressure = bmp.readPressure() / 100.0F;     // hPa
  float altitude = bmp.readAltitude(seaLevelPressure); // relative altitude

  Serial.print("Temp: ");
  Serial.print(temperature);
  Serial.print(" °C, Pressure: ");
  Serial.print(pressure);
  Serial.print(" hPa, Altitude (relative): ");
  Serial.print(altitude);
  Serial.println(" m");

  delay(1000);
}
