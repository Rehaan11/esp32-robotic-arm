
# ESP32 Hand Gesture-Controlled Robotic Arm

An embedded systems project using the ESP32, ESP-IDF, C programming, five servo motors, potentiometers, and an MPU6050 accelerometer/gyroscope.

The project is being developed in Wokwi to test the firmware and circuit before physical hardware implementation.

## Project Overview

The goal of this project is to develop a robotic arm whose movements can be controlled using hand orientation measured by an MPU6050 sensor.

The ESP32 acts as the main controller. It reads analog inputs from five potentiometers, generates PWM signals for five servo motors, and communicates with the MPU6050 through the I2C interface.

### Current Implementation

- ESP32 firmware developed using ESP-IDF in C.
- Five potentiometers independently control five servo positions.
- Five servo PWM channels are configured using the ESP32 LEDC peripheral.
- MPU6050 communication and accelerometer register reading are implemented.
- Accelerometer-based roll and pitch calculations are included.
- Wokwi is used for circuit simulation and firmware debugging.

**Development status:** Potentiometer-to-servo control is implemented. MPU6050 tilt response and integration of roll/pitch with servo control are still being validated.

## Features

- Independent control of five servo motors.
- ADC-based potentiometer input acquisition.
- PWM generation using the ESP32 LEDC peripheral.
- I2C communication with the MPU6050.
- Accelerometer-based roll and pitch estimation.
- Serial Monitor output for debugging.
- Simulation using Wokwi.

## Hardware Components

| Component | Quantity | Purpose |
|---|---:|---|
| ESP32 DevKit | 1 | Main controller |
| MPU6050 module | 1 | Acceleration and angular velocity sensing |
| Servo motors | 5 | Robotic arm joint actuation |
| Potentiometers | 5 | Manual joint-position inputs |
| Connecting wires | As required | Circuit connections |

The project is currently simulated in Wokwi. Physical hardware implementation and validation are separate steps.

## Software and Tools

- C programming language
- ESP-IDF framework
- Wokwi ESP32 simulator
- Git and GitHub
- ESP-IDF ADC driver
- ESP-IDF I2C driver
- ESP-IDF LEDC PWM driver

## System Architecture

```text
Potentiometers
      |
      v
ESP32 ADC -----> Angle Mapping
                      |
                      v
                LEDC PWM Output
                      |
                      v
                Five Servos

MPU6050
   |
   | I2C (SDA, SCL)
   v
  ESP32
   |
   v
Accelerometer Data
   |
   v
Roll and Pitch Calculation
   |
   v
Serial Monitor
```

The MPU6050-to-servo control path is a planned integration step and is not yet complete.

## Pin Connections

### Servo Motors

| Servo | Function | ESP32 GPIO |
|---|---|---:|
| Servo 1 | Base | GPIO18 |
| Servo 2 | Gripper | GPIO19 |
| Servo 3 | Elbow | GPIO23 |
| Servo 4 | Shoulder | GPIO25 |
| Servo 5 | Wrist | GPIO26 |

### Potentiometers

| Potentiometer | ESP32 GPIO | ADC Channel |
|---|---:|---|
| POT1 | GPIO32 | ADC1_CHANNEL_4 |
| POT2 | GPIO33 | ADC1_CHANNEL_5 |
| POT3 | GPIO34 | ADC1_CHANNEL_6 |
| POT4 | GPIO39 (VN) | ADC1_CHANNEL_3 |
| POT5 | GPIO36 (VP) | ADC1_CHANNEL_0 |

Each potentiometer connects to 3.3 V, GND, and its assigned signal pin.

### MPU6050

| MPU6050 Pin | ESP32 Connection |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO21 |
| SCL | GPIO22 |

## How It Works

### 1. Read Potentiometers

The ESP32 reads five analog inputs using ADC1. The 12-bit ADC readings range from 0 to 4095.

### 2. Convert Readings to Angles

Each ADC value is mapped to a nominal servo target angle between 0 and 180 degrees:

`angle = (ADC_reading * 180) / 4095`

### 3. Generate PWM Signals

The LEDC peripheral generates servo control pulses at 50 Hz. The current firmware maps 0–180 degrees to pulse widths of approximately 500–2500 microseconds.

Actual pulse limits should be verified for the selected servo before physical operation.

### 4. Read MPU6050

The ESP32 reads accelerometer registers through I2C and calculates roll and pitch from the acceleration values.

Accelerometer-based tilt estimates are most reliable when linear acceleration is small. Gyroscope fusion and filtering are not yet implemented.

### 5. Future Gesture Integration

The next development step is to map calibrated roll and pitch angles to selected servo targets, with angle limits and smoothing.

## Project Structure

```text
esp32-hand-gesture-robotic-arm/
├── main/
│   ├── main.c
│   └── CMakeLists.txt
├── CMakeLists.txt
├── diagram.json
├── wokwi.toml
├── README.md
└── .gitignore
```

The structure above describes the intended repository layout. Include the configuration files used by your actual working project.

## Running the Simulation

1. Open the project in Wokwi.
2. Verify the ESP32, potentiometer, servo, and MPU6050 connections.
3. Build and start the ESP-IDF simulation.
4. Open the Serial Monitor.
5. Rotate each potentiometer and check that its corresponding servo responds.
6. Inspect the MPU6050 identity and accelerometer readings.
7. Verify that the raw acceleration values change when the simulated sensor orientation changes.

For a local ESP-IDF build, use an installed and configured ESP-IDF environment and build from the project root:

```bash
idf.py build
```

The Wokwi project configuration and ESP-IDF environment must match the firmware version and APIs used by the source code.

## Future Improvements

- Validate MPU6050 roll and pitch response in simulation and hardware.
- Integrate hand tilt with selected servo joints.
- Add angle calibration, filtering, and movement smoothing.
- Add gesture thresholds and safe servo limits.
- Test the complete robotic arm using physical hardware.
- Document measured results and demonstration videos.

## Applications

- Embedded systems learning
- Robotics and actuator control
- Sensor interfacing using I2C
- ADC and PWM peripheral programming
- Microcontroller firmware development

## Author

**Rehaan Rameez**

Embedded Systems Project


## License

A license has not yet been selected. Add a license file if you want to specify the terms under which others may use, modify, and distribute this project.
