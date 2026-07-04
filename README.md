# ESP32 1-Axis Active Stabilization System

**Real-time gyroscopic stabilization using a complementary filter — ESP32 + MPU6050 + SG90 Servo**

---

## Overview

A 1-axis active stabilization rig that keeps a platform level in real time by fusing accelerometer and gyroscope data through a complementary filter. The system reads tilt angle from an MPU6050 IMU, computes a clean filtered angle, and counter-rotates an SG90 servo to cancel out disturbances.

Includes startup gyroscope bias calibration — averaging 500 readings at rest to subtract the constant offset error before the filter runs, significantly reducing drift especially at low speeds.

**Potential applications:**
- Satellite solar array single-axis sun tracking
- Marine communication dish stabilization against vessel rocking
- Camera gimbal stabilization (1-axis)
- Rocket or drone attitude reference platform

---

## How It Works

### The Sensor Fusion Problem

Two sensors, two problems:

**Accelerometer** — gives a stable absolute angle using `atan2(ay, az)` but is noisy during vibration and sudden movements. It acts as a **low-pass filter** — reliable for slow, steady-state angles.

**Gyroscope** — tracks fast angular changes accurately by integrating rotational speed over time (`angle += gyroRate × dt`) but accumulates **drift** over time due to a small constant bias error in the raw output.

Neither sensor alone is sufficient for real-time stabilization.

### The Complementary Filter

Combines both sensors to cover the full frequency spectrum of motion:

```
filteredAngle = 0.98 × (filteredAngle + gyroRate × dt) + 0.02 × accelAngle
```

- **98%** gyroscope — captures fast angular changes cleanly
- **2%** accelerometer — continuously corrects slow drift with a stable reference

The 0.98/0.02 split treats the gyroscope as a high-pass filter and the accelerometer as a low-pass filter. Together they cancel each other's weaknesses.

### Gyroscope Bias Calibration

Even before drift accumulates, the gyroscope has a small constant offset at rest — it never reads exactly zero when stationary. At startup, the system averages 500 gyroscope readings over ~1 second to calculate this bias and subtracts it before the filter runs:

```cpp
float gyroRateX = (raw_gx - gyroXBias) / 131.0;
```

This tightens angle accuracy significantly, especially at low speeds.

### Stabilization Command

The filtered angle is inverted and applied to the servo:
```cpp
targetServoAngle = SERVO_CENTER - filteredAngleX;
```

If the platform tilts +15°, the servo rotates -15° to compensate and keep the platform level.

---

## Components

| Component | Specification | Purpose |
|---|---|---|
| ESP32 Dev Board | Any 30-pin variant | Main processor |
| MPU6050 | 6-DOF IMU (I2C, 0x68) | Accelerometer + Gyroscope |
| SG90 Servo | 180° range, 50Hz PWM | Stabilization actuator |
| Jumper wires | Male-to-Male / Male-to-Female | Connections |
| Breadboard | Standard 830-point | Prototyping |
| USB cable | Micro-USB or Type-C | Power + programming |

---

## Wiring

### MPU6050 → ESP32

| MPU6050 Pin | ESP32 Pin | Description |
|---|---|---|
| VCC | 3.3V | Power (use 3.3V not 5V) |
| GND | GND | Ground |
| SDA | GPIO 21 | I2C Data |
| SCL | GPIO 22 | I2C Clock |
| AD0 | GND | I2C address = 0x68 |
| INT | Not connected | Interrupt (unused) |

### SG90 Servo → ESP32

| Servo Wire | ESP32 Pin | Description |
|---|---|---|
| Red | 5V (VIN) | Power |
| Brown / Black | GND | Ground |
| Orange / Yellow | GPIO 13 | PWM Signal |

### Wiring Diagram

```
ESP32                MPU6050
┌──────────┐        ┌──────────┐
│  3.3V ───┼────────┼─ VCC     │
│  GND  ───┼────────┼─ GND     │
│  GPIO21──┼────────┼─ SDA     │
│  GPIO22──┼────────┼─ SCL     │
│          │        │          |
└──────────┘        └──────────┘

ESP32                SG90 Servo
┌──────────┐        ┌──────────┐
│  5V   ───┼────────┼─ Red     │
│  GND  ───┼────────┼─ Brown   │
│  GPIO13──┼────────┼─ Orange  │
└──────────┘        └──────────┘
```

> **Important:** Power the servo from the ESP32's 5V (VIN) pin, not 3.3V. The MPU6050 must be powered from 3.3V — connecting it to 5V will damage it.

---

## Libraries

Install all libraries via Arduino IDE Library Manager (`Sketch → Include Library → Manage Libraries`):

| Library | Author | Install Name | Purpose |
|---|---|---|---|
| MPU6050 | Electronic Cats / Jeff Rowberg | `MPU6050` | IMU driver |
| I2Cdev | Jeff Rowberg | `I2Cdev` | I2C device abstraction |
| ESP32Servo | Kevin Harrington | `ESP32Servo` | Servo PWM for ESP32 |

> **Note:** The standard Arduino `Servo.h` library does not work on ESP32. You must use `ESP32Servo`.

---

## Setup & Upload

**1. Install Arduino IDE** from `arduino.cc`

**2. Add ESP32 board support:**
- Go to `File → Preferences`
- Add to Additional Board Manager URLs:
```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```
- Go to `Tools → Board → Board Manager` → search `esp32` → install

**3. Install libraries** — see Libraries section above

**4. Select board:**
- `Tools → Board → ESP32 Arduino → ESP32 Dev Module`
- `Tools → Port → [your COM port]`

**5. Upload the sketch** and open Serial Monitor at 115200 baud

**6. Calibration:** Keep the sensor completely still for ~1 second after power-on while calibration runs. You will see:
```
--- SYSTEM STARTING ---
Initializing MPU6050...
Calibrating gyroscope — keep sensor still...
Gyro bias calibrated: 12.34
System Ready!
```

---

## Serial Plotter

Open `Tools → Serial Plotter` at 115200 baud to visualize:
- `Accel_Angle` — raw accelerometer angle (noisy)
- `Filtered_Gyro_Angle` — complementary filter output (clean)

The difference between the two lines visually demonstrates why sensor fusion is necessary.

---

## Configuration

All tunable parameters are at the top of the sketch:

| Parameter | Default | Description |
|---|---|---|
| `SERVO_PIN` | `13` | GPIO pin for servo signal |
| `SERVO_CENTER` | `90` | Neutral servo position (degrees) |
| `0.98 / 0.02` | Fixed | Complementary filter coefficients |
| `500` samples | Fixed | Calibration averaging count |
| `delay(3)` | 3ms | Loop rate (~100Hz update) |

---


## About

**Syed Talha Jamal**
Computer Engineering Student — SSUET Karachi, Pakistan

- GitHub: [github.com/talha436dev](https://github.com/talha436dev)
- LinkedIn: [linkedin.com/in/syed-talha-jamal-901622391](https://linkedin.com/in/syed-talha-jamal-901622391)
