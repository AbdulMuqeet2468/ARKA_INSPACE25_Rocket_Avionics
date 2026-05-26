#  Avionics System — ISRO IN-Space Rocketry Challenge 2025

**Author:** M. Abdul Muqeet · [LinkedIn](https://www.linkedin.com/in/mabdulmuqeet/)  


---

<img width="1280" height="720" alt="Image" src="https://github.com/user-attachments/assets/96aabbf9-ee77-4deb-b326-e04f4cc41300" />

> **A.R.K.A Rocket**


---

<img width="1599" height="720" alt="Image" src="https://github.com/user-attachments/assets/8d1a47c0-18d0-41c4-830c-957f3df8f3a5" />

> **Avionics Bay**

## File Structure

```
avionics/
├── LoRaTX_code.ino      # Main rocket transmitter code (all sensors + LoRa TX + servo)
├── LoRaRX_code.ino      # Ground station receiver code (LoRa RX → Serial)
├── mpu_test.ino         # Standalone MPU6500 test
├── bmp_test.ino         # Standalone BMP280 test
├── gps_test.ino         # Standalone GPS test
├── servo_test.ino       # Standalone servo test
└── AvionicsReadMe.md    # Original hardware connection reference
```


## Table of Contents

1. [System Overview](#system-overview)
2. [Hardware Architecture](#hardware-architecture)
3. [Pin Connections](#pin-connections)
4. [Software Modules](#software-modules)
5. [Data Telemetry Format](#data-telemetry-format)
6. [Algorithms Explained](#algorithms-explained)
7. [Servo Deployment Logic](#servo-deployment-logic)
8. [Setup & Flashing Guide](#setup--flashing-guide)
9. [Library Dependencies](#library-dependencies)
10. [Debugging Guide](#debugging-guide)
11. [Known Limitations & Caveats](#known-limitations--caveats)

---

## System Overview

This avionics system is built with the primary goal of:

- Acquiring flight data in real-time (GPS, altitude, orientation, acceleration, velocity)
- Wirelessly transmitting that data over LoRa to a ground station receiver
- Autonomously deploying a parachute/recovery servo at a set altitude threshold

The system is split into two ESP32 nodes:

| Node | Role | Key Components |
|------|------|----------------|
| **TX (Transmitter)** | Onboard the rocket | MPU6500, BMP280, Neo-6M GPS, LoRa SX1278, Servo (MG996R) |
| **RX (Receiver)** | Ground station | LoRa SX1278, Serial output to PC/GUI |

---

## Hardware Architecture

```
┌──────────────────────────────────────────────┐
│              ROCKET (TX ESP32)               │
│                                              │
│  ┌──────────┐  ┌──────────┐  ┌───────────┐   │
│  │ MPU6500  │  │  BMP280  │  │  Neo-6M   │   │
│  │  (IMU)   │  │(Baro/Alt)│  │  (GPS)    │   │
│  └────┬─────┘  └────┬─────┘  └─────┬─────┘   │
│       │  I2C        │  I2C          │ UART2  │
│       └─────────────┴──────────────┘         │
│                      │                       │
│               ┌──────┴──────┐                │
│               │   ESP32     │─GPIO25-► Servo │
│               └──────┬──────┘                │
│                      │ SPI                   │
│               ┌──────┴──────┐                │
│               │  LoRa TX    │ ))))           │
│               └─────────────┘                │
└──────────────────────────────────────────────┘

                     ))))   433 MHz RF Link

┌──────────────────────────────────────────────┐
│            GROUND STATION (RX ESP32)         │
│                                              │
│               ┌─────────────┐                │
│               │  LoRa RX    │                │
│               └──────┬──────┘                │
│                      │ SPI                   │
│               ┌──────┴──────┐                │
│               │   ESP32     │── USB ─► PC/GUI│
│               └─────────────┘                │
└──────────────────────────────────────────────┘
```

---

## Pin Connections

### TX ESP32 (Rocket Side)

#### MPU6500 (IMU) — I2C
| MPU6500 Pin | ESP32 Pin |
|-------------|-----------|
| VCC | 3.3V |
| GND | GND |
| SCL | GPIO 22 |
| SDA | GPIO 21 |

#### BMP280 (Barometer) — I2C
| BMP280 Pin | ESP32 Pin |
|------------|-----------|
| VCC | 3.3V |
| GND | GND |
| SCL | GPIO 22 |
| SDA | GPIO 21 |

> Both MPU6500 and BMP280 share the same I2C bus (SDA=21, SCL=22). They have different I2C addresses: MPU6500 at `0x68`, BMP280 at `0x76` (try `0x77` if unresponsive).

#### Neo-6M GPS — UART
| GPS Pin | ESP32 Pin |
|---------|-----------|
| VCC | 3.3V |
| GND | GND |
| TX | GPIO 16 (RX2) |
| RX | GPIO 17 (TX2) |

#### LoRa SX1278 (TX) — SPI
| LoRa Pin | ESP32 Pin |
|----------|-----------|
| 3.3V | 3.3V |
| GND | GND |
| RST | GPIO 27 |
| DIO0 | GPIO 26 |
| NSS (CS) | GPIO 5 |
| MOSI | GPIO 23 |
| MISO | GPIO 19 |
| SCK | GPIO 18 |

#### Servo
| Servo Wire | Connection |
|------------|------------|
| Red (VCC) | 5V from regulator |
| Brown (GND) | GND of regulator |
| Yellow (Signal) | GPIO 25 |

> ⚠️ **Do not power the servo from the ESP32's 3.3V pin.** Servos draw significant current and will brown-out the ESP32. Always use a dedicated 5V regulator output.

---

### RX ESP32 (Ground Station)

#### LoRa SX1278 (RX) — SPI
| LoRa Pin | ESP32 Pin |
|----------|-----------|
| 3.3V | 3.3V |
| GND | GND |
| RST | GPIO 4 |
| DIO0 | GPIO 2 |
| NSS (CS) | GPIO 23 |
| MOSI | GPIO 27 |
| MISO | GPIO 19 |
| SCK | GPIO 18 |

---

## Software Modules

### `LoRaTX_code.ino` — Main Rocket Code

The transmitter is the heart of the system. It:

1. **Reads IMU data** from the MPU6500 at ~20 Hz via I2C
2. **Reads barometric data** from the BMP280 at 1 Hz
3. **Reads GPS data** from Neo-6M via UART2 at 1 Hz
4. **Computes orientation** (pitch, roll, yaw) using a complementary filter
5. **Computes world-frame velocity** via dead reckoning from accelerometer integration
6. **Checks servo deployment** condition (altitude ≥ 1000 m)
7. **Serializes all data** into a CSV string and transmits over LoRa at 1 Hz

### `LoRaRX_code.ino` — Ground Station Code

Listens for incoming LoRa packets and prints the raw CSV to Serial. This output is intended to be consumed by a GUI application for real-time visualization.

### Individual Test Files

| File | Purpose |
|------|---------|
| `mpu_test.ino` | Tests MPU6500: prints pitch, roll, yaw, world-frame acceleration, and estimated velocity |
| `bmp_test.ino` | Tests BMP280: prints temperature, pressure, and relative altitude |
| `gps_test.ino` | Tests Neo-6M GPS: prints full fix info (lat, lon, altitude, speed, satellites) |
| `servo_test.ino` | Tests servo: attaches to GPIO 25, writes to `servo_off` position at startup |

---

## Data Telemetry Format

All data is transmitted as a single comma-separated line over LoRa:

```
latitude,longitude,altitude,pressure,temperature,pitch,roll,yaw,ax,ay,az,vx,vy,vz,servo_status
```

| Field | Unit | Description |
|-------|------|-------------|
| `latitude` | degrees (6 decimal places) | GPS latitude |
| `longitude` | degrees (6 decimal places) | GPS longitude |
| `altitude` | metres | Relative altitude from BMP280 (calibrated at boot) |
| `pressure` | hPa | Barometric pressure |
| `temperature` | °C | Ambient temperature from BMP280 |
| `pitch` | degrees | Nose-up (+) / nose-down (−) |
| `roll` | degrees | Right-wing-down (+) |
| `yaw` | degrees | Integrated from gyro (drifts over time, no magnetometer correction) |
| `ax` | m/s² | World-frame X acceleration (gravity removed) |
| `ay` | m/s² | World-frame Y acceleration (gravity removed) |
| `az` | m/s² | World-frame Z acceleration (gravity removed) |
| `vx` | m/s | Integrated velocity, X axis |
| `vy` | m/s | Integrated velocity, Y axis |
| `vz` | m/s | Integrated velocity, Z axis |
| `servo_status` | 0 or 1 | 0 = parachute not deployed, 1 = deployed |

**Example packet:**

```
17.385044,78.486671,312.45,988.21,28.40,2.31,-0.87,14.52,0.12,-0.05,9.72,0.002,-0.001,0.034,0
```

---

## Algorithms Explained

### 1. Orientation Estimation (Complementary Filter)

Pitch and roll are estimated by fusing the accelerometer (absolute but noisy) and the gyroscope (smooth but drifts) using a complementary filter:

```
pitch = α × (pitch + gyr_x × dt) + (1 − α) × acc_pitch
roll  = α × (roll  + gyr_y × dt) + (1 − α) × acc_roll
```

`α = 0.96` in the TX code (0.98 in the standalone test). A higher α means more trust in the gyroscope. Yaw is pure gyro integration and will drift without a magnetometer.

### 2. Gravity Removal & World-Frame Acceleration

The body-frame accelerometer readings (in g) are rotated to the world frame using a full 3D rotation matrix built from pitch/roll/yaw angles. Gravity (9.81 m/s²) is then subtracted from the world Z axis:

```
az_world = R * acc_z * G - G
```

This gives the net linear acceleration of the rocket in the world frame.

### 3. Low-Pass Filter on Acceleration (TX only)

An exponential moving average is applied to the world-frame accelerations before velocity integration to suppress high-frequency noise:

```
ax_filt = 0.85 × ax_filt + 0.15 × ax_world
```

### 4. Velocity Integration (Dead Reckoning)

Velocity is computed using the trapezoidal integration rule for numerical accuracy:

```
vx += 0.5 × (prev_ax + ax_filt) × dt
```

### 5. Stationary Detection & Velocity Reset

To prevent velocity drift accumulation while the rocket is stationary (on the pad or after landing), the system checks if the total accelerometer magnitude is approximately 1 g and all gyro rates are below a threshold. If this condition holds for `STATIONARY_COUNT_TO_RESET = 5` consecutive samples, velocity is forcibly reset to zero.

| Parameter | TX Value | Test Value |
|-----------|----------|------------|
| Accel magnitude tolerance | ±0.02 g | ±0.03 g |
| Gyro tolerance | 1.5 °/s | 2.0 °/s |
| Consecutive samples to reset | 5 | 3 |

### 6. Relative Altitude (BMP280)

At boot, the current barometric pressure is stored as the sea-level reference. All subsequent altitude readings are relative to this baseline using the hypsometric formula (handled internally by the Adafruit BMP280 library). This is appropriate for relative altitude tracking during a flight but does not give absolute MSL altitude.

---

## Servo Deployment Logic

The servo is used to deploy the recovery system (parachute mechanism). Deployment is one-way and irreversible in flight:

```cpp
if (altitude >= 1000 && servo_status == 0) {
    myservo.write(servo_on);   // default: 180°
    servo_status = 1;
}
```

**Before installing:**
1. Power the servo and manually test rotation using `servo_test.ino`
2. Determine which angle (`servo_on = 180`, `servo_off = 0`) physically deploys your recovery mechanism
3. Swap `servo_on` / `servo_off` values if the rotation direction is wrong
4. Adjust the `1000` metre threshold to match your apogee design

> The deployment altitude (1000 m) **must be calibrated** to your rocket's expected apogee before flight.

---

## Setup & Flashing Guide

### Prerequisites

- Arduino IDE 1.8+ or Arduino IDE 2.x
- ESP32 board package installed (`https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`)
- All libraries installed (see below)

### Steps

1. Open `LoRaTX_code.ino` in Arduino IDE
2. Select board: **ESP32 Dev Module** (or your specific variant)
3. Select the correct COM port
4. Upload speed: 115200 baud
5. Open Serial Monitor at **115200 baud** to verify sensor readings
6. Repeat for the RX ESP32 with `LoRaRX_code.ino`

### Calibration at Boot

The BMP280 calibrates its sea-level pressure reference on the very first reading at startup. Always power on the TX unit at ground level and allow it a few seconds before moving it.

The MPU6500 auto-calibrates offsets via `myMPU6500.autoOffsets()` — keep the unit completely still and flat for the first ~2 seconds after power-on.

---

## Library Dependencies

Install all of the following via Arduino IDE Library Manager:

| Library | Author | Install Name |
|---------|--------|-------------|
| TinyGPSPlus | Mikal Hart | `TinyGPSPlus` |
| Adafruit Unified Sensor | Adafruit | `Adafruit Unified Sensor` |
| Adafruit BMP280 Library | Adafruit | `Adafruit BMP280 Library` |
| MPU6500_WE | Wolfgang Ewald | `MPU6500_WE` |
| LoRa | Sandeep Mistry | `LoRa` |
| ESP32Servo | Kevin Harrington | `ESP32Servo` |

> If `MPU6500_WE` is not found in the Library Manager, alternatives are:
> - `MPU9250_WE` (same author, superset)
> - `MPU6050` by Electronic Cats
> - `Adafruit MPU6050` by Adafruit
>
> If using an alternative library, the function calls (`getGValues()`, `getGyrValues()`, etc.) will differ — update accordingly.

---

## Debugging Guide

### GPS Not Locking

- Check wiring: GPS TX → ESP32 RX2 (GPIO16), GPS RX → ESP32 TX2 (GPIO17)
- The GPS `setup()` times out after 5 seconds and halts if no characters arrive on Serial2 — if it halts instantly, wiring is wrong
- GPS fix can take 30–120 seconds outdoors (cold start). Always test outside with clear sky view
- If `gps.location.isValid()` returns false, data will be transmitted as `0.000000,0.000000` — the GUI should handle this gracefully

### BMP280 Not Found

- Default I2C address is `0x76`. Some modules use `0x77` — change in `bmp.begin(0x76)` if needed
- Verify SDA/SCL wiring is correct and pull-up resistors are present (or use the ones on the module breakout board)
- If both MPU6500 and BMP280 fail, check the I2C bus with an I2C scanner sketch

### MPU6500 Not Found

- Default I2C address is `0x68`. If AD0 pin is pulled HIGH, address becomes `0x69` — update `#define MPU6500_ADDR` accordingly
- Run `autoOffsets()` only when the sensor is still and flat
- High vibration during flight will add noise — the low-pass filter (α = 0.85) reduces this but does not eliminate it

### LoRa Not Initializing

- Double-check all 8 SPI pin connections (MOSI, MISO, SCK, NSS, DIO0, RST, 3.3V, GND)
- Ensure the LoRa module is powered from 3.3V, not 5V
- TX and RX must be on the same frequency (`433E6`) — also verify they are not transmitting simultaneously (this system is simplex: TX always transmits, RX always listens)

### Servo Not Moving

- Verify the signal wire (yellow) goes to GPIO 25 and the power comes from a 5V source, not the ESP32
- Test with `servo_test.ino` first. If it doesn't move there, it's a wiring/power issue
- If using a different GPIO, update `#define SERVO_PIN` in both `servo_test.ino` and `LoRaTX_code.ino`

### Velocity Drift on Ground

- If velocity readings don't return to zero when stationary, tighten the stationary detection thresholds: lower `ACC_MAG_TOL_G` and `GYR_TOL_DEG`
- Ensure the MPU6500 completed `autoOffsets()` correctly at boot (unit must be perfectly still)
- Velocity dead-reckoning is inherently subject to drift. These readings are indicative, not navigation-grade

### Serial Monitor Shows Garbage

- Ensure Serial Monitor baud rate matches `Serial.begin(115200)`
- On some ESP32 boards, the first few bytes after boot are garbage — this is normal

---

## Known Limitations & Caveats

| Limitation | Detail |
|-----------|--------|
| **Yaw drift** | Yaw is integrated from the gyroscope with no magnetometer correction. It will drift significantly over a full flight. Pitch and roll are stable due to accelerometer correction. |
| **Velocity accuracy** | Dead-reckoning from a consumer-grade MEMS IMU accumulates error rapidly. Velocity values are useful for detecting motion phases but are not navigation-grade. |
| **Altitude source** | Altitude in the telemetry CSV comes from the BMP280 (barometric), not GPS. GPS altitude is separately available but not currently included in the CSV payload. |
| **LoRa packet rate** | Transmission rate is 1 Hz (`delay(1000)` in `loop()`). MPU data is computed at a higher rate internally but only the latest sample is sent. |
| **Servo is one-way** | Once `servo_status = 1`, there is no software path to retract. This is intentional for safety. |
| **No CRC on LoRa payload** | The CSV string has no checksum. Corrupted packets will parse silently with wrong values. The GUI should implement sanity checks. |
| **GPS cold start** | If GPS has no fix at liftoff, latitude and longitude in the telemetry will be `0.000000`. Plan for this in the GUI. |
| **BMP280 altitude is relative** | The altitude reference is set at boot time ground level, not mean sea level (MSL). Absolute altitude requires the actual QNH pressure for your launch site. |

---



---

*Built for ISRO IN-Space Rocketry Challenge 2025 (Team ARKA - Chaitanya ASTRA - CBIT Hyderabad).*
