# AGSG — Self-Driving Vehicle (WRO Future Engineers 2026)

**Team:** AGSG (Portugal) — Joana Silva, Simão Freire · **Coach:** Tiago Severino

## Table of contents

1. [Introduction](#1-introduction)
2. [Vehicle Hardware](#2-vehicle-hardware)
3. [Software Architecture](#3-software-architecture)
4. [Build and Upload](#4-build-and-upload)
5. [Repository Structure](#5-repository-structure)
6. [Reproducibility](#6-reproducibility)

---

## 1. Introduction

This repository contains the full engineering documentation and control software of our self-driving vehicle, developed for the **WRO Future Engineers** category in the **2026** season. The vehicle is a small **Ackermann-style car** that drives autonomously around the field, follows the outer lane, obeys traffic signs, avoids obstacles and parks itself.

The system is built around three programmable boards:

| Board | Role |
|-------|------|
| **Raspberry Pi Pico 2** (main) | Motor/servo control, odometry, IMU, navigation and decision logic |
| **Raspberry Pi Pico 1** (sonar slave) | Reads the four ultrasonic distance sensors and streams them over UART |
| **OpenMV H7 R1** (camera) | Detects the coloured traffic signs and streams the detections over UART |

All control software is written in **MicroPython**.

## 2. Vehicle Hardware

### 2.1 Components

| Component | Model / specification | Purpose |
|-----------|-----------------------|---------|
| Main controller | Raspberry Pi Pico 2 (RP2350) | navigation, control, sensor fusion |
| Sonar controller | Raspberry Pi Pico 1 (RP2040) | reads the 4 ultrasonic sensors |
| Camera | OpenMV H7 R1 | traffic-sign colour detection |
| Drive motor | N20 1000 RPM 12 V DC, with quadrature encoder | propulsion + odometry |
| Gear / differential | 3:5 reduction + metal RC differential gear set (Wltoys 144001 type) | rear axle, cornering |
| Motor driver | TB6612FNG H-bridge | motor speed and direction |
| Steering | Savox SH-0255 MG servo | Ackermann front steering |
| IMU | BNO085 | heading (game rotation vector) |
| Distance sensors | 4× SRF04 ultrasonic (front, rear, left, right) | wall/obstacle distance |
| Communication | HC-06 Bluetooth module | wireless telemetry for debugging only (TX only; removed during competition rounds) |
| Battery | 3× 18650 Li-ion in series (11.1 V, 1800 mAh) | power source |
| Regulators | LM2596 (buck), 2× L7805CV | power distribution |

### 2.2 Power Architecture

The battery (3× 18650 in series, ≈11.1 V) feeds three independent rails:

- **LM2596** buck converter → constant **11 V** → **TB6612FNG** (motor power).
- **L7805CV #1** → **5 V** → **steering servo**.
- **L7805CV #2** → **5 V** → both Picos, BNO085, HC-06, SRF04 sensors and the OpenMV camera.

The motor and the logic/sensor rails are separated so that motor current spikes do not disturb the sensitive electronics. The full pin-level wiring diagram is available in `schemes/`.

### 2.3 Dimensions and Mechanical Design

The vehicle measures **287 mm (length) × 61 mm (width) × 170 mm (height)**. We aimed to keep the vehicle compact so that it manoeuvres easily around the field without hitting walls or obstacles.

The front of the body is a raised section that stays **above** the walls and obstacles, while the lower part of the chassis — the part that interacts with the field elements — is only **127 mm** long. This keeps the vehicle short at wall level and gives extra clearance over low obstacles.

The drivetrain is a single rear **DC motor (N20, 1000 RPM) with a quadrature encoder**, driving the rear axle through a **3:5 gear reduction** and a **metal RC differential**, which lets the two rear wheels rotate at different speeds through corners. The front wheels use **Ackermann steering** driven by a single **servo**. The encoder provides odometry (distance travelled), and the BNO085 IMU provides heading, so the robot navigates with a closed-loop position estimate.

## 3. Software Architecture

### 3.1 Overview

The main controller (Pico 2) runs a real-time **background control loop on the second core** (MicroPython `_thread`), while the navigation state machine runs on the first core. The background loop continuously:

1. reads the distance sensors (over UART from the sonar slave),
2. reads the camera detections (over UART1 from the OpenMV),
3. integrates **odometry** (encoder distance + IMU heading),
4. runs the **cruise control** (motor speed regulation),
5. sends **telemetry** over Bluetooth,
6. applies **sonar-based odometry corrections**.

### 3.2 Main controller modules (`src/`)

| Module | Responsibility | Related hardware |
|--------|----------------|------------------|
| `main.py` | Entry point; runs the obstacle challenge | — |
| `init.py` | Hardware init (300 MHz clock, UARTs, camera, servo); starts the background thread | all |
| `obstacle_challenge.py` | The challenge **state machine** (parking, lanes, traffic signs, obstacles) | all |
| `move_coordinates.py` | Navigation primitives: `move_coordinates`, `rotate_angle`, `rotate_coordinates` (odometry + PID steering) | motor, servo, encoder, IMU |
| `background.py` | Background control loop (sensors, odometry, cruise control, telemetry) | sonar, camera, encoder, IMU, motor |
| `state.py` | Shared robot state (odometry, lane, direction) | — |
| `field.py` | Field model (4 lanes, 3 zones, traffic-signal tracking) | — |
| `camera.py` | Parses OpenMV traffic-sign detections (UART1) | OpenMV |
| `distance.py` | Parses sonar distance over a framed protocol (header 0xA5 + XOR checksum) | sonar slave |
| `encoder.py` | Quadrature encoder with dynamic debounce | encoder |
| `motor.py` | Motor PWM and direction (20 kHz) | TB6612FNG |
| `servo.py` | Steering servo (50 Hz PWM) | servo |
| `compass.py` | BNO085 heading (game rotation vector, no magnetometer) | BNO085 |
| `bno08x.py` | BNO08x driver library | BNO085 |
| `pid.py` | Generic PID controller (used for steering) | — |
| `telemetry.py` | Telemetry frame (CSV + XOR checksum) over UART → Bluetooth | HC-06 |
| `uart.py` | Shared UART helper | — |

### 3.3 Sonar slave (`src/SonarSlave/`)

The sonar slave (Pico 1) reads the four SRF04 ultrasonic sensors and transmits the four distances over UART0 using a framed protocol (header byte `0xA5`, four little-endian `uint16` values scaled by 10, and an XOR checksum). This framing lets the receiver re-synchronise if a byte is ever lost.

### 3.4 Camera (`src/Camera/`)

The OpenMV H7 runs a colour-blob detector that finds the red and green traffic signs in the image and streams `CAM:<cx>,<bottom_y>,<COLOR>` lines over UART. The main controller maps each detection to the corresponding traffic-sign position on the field.

### 3.5 Concurrency and real-time control

The vehicle uses **two cores** of the main Pico:

- **Core 0** runs the navigation state machine (`obstacle_challenge`), which sets target speed/steering and reads the position estimate.
- **Core 1** runs the `background` loop, which reads sensors at fixed rates (distance 50 Hz, camera 50 Hz, odometry 100 Hz, speed control 10 Hz) and drives the motor PWM through the cruise controller.

The cruise controller is an incremental speed controller with acceleration/deceleration limits, and the steering uses a **PID** controller on the cross-track error. The encoder uses a **dynamic debounce** whose window scales with the current speed, and the motor runs at **20 kHz PWM** so switching noise stays above the encoder band.

### 3.6 How the robot drives

The control is a closed loop. The state machine (`obstacle_challenge`) sets a target speed and target heading; the background loop continuously integrates odometry — distance from the encoder (`counts / (PPR / gear ratio) × wheel circumference`) and heading from the BNO085 IMU — and a **PID** controller steers toward the target heading while the **cruise controller** regulates motor speed with acceleration and deceleration limits. The sonars (distance) and the camera (traffic signs) feed events back to the state machine, which switches state whenever a milestone is reached (a wall ahead, a traffic sign, or the parking spot).

## 4. Build and Upload

All firmware is flashed with **MicroPython**:

1. Flash MicroPython onto the Pico 2 (main) and Pico 1 (sonar slave).
2. Upload the files in `src/` to the **Pico 2** (using the MicroPico VS Code extension, Thonny, or `mpremote`). `main.py` is the entry point.
3. Upload the contents of `src/SonarSlave/` to the **Pico 1** (`main.py` runs the sonar loop).
4. Flash the OpenMV firmware in `src/Camera/main.py` onto the **OpenMV H7**.
5. Power on: the main controller runs `main.py` automatically and starts the obstacle challenge.

The only external dependency is the `bno08x.py` driver, which is included in the repository.

## 5. Repository Structure

```
├── models/        # CAD / 3D-printable and CNC files
├── other/         # extra files (hardware specs, upload notes, datasets)
├── schemes/       # wiring diagrams (electromechanical)
├── src/           # main controller software (MicroPython)
│   └── SonarSlave/  # sonar controller firmware
│   └── Camera/      # OpenMV camera firmware
├── t-photos/      # team photos
├── v-photos/      # vehicle photos (all sides, top, bottom)
└── video/         # link to the driving-demonstration video
```

## 6. Reproducibility

To reproduce this vehicle, another team needs the hardware listed in §2.1, the wiring diagram in `schemes/`, the CAD files in `models/`, and the source code in `src/`. The README and code comments document the module responsibilities and the build/upload process, so the robot can be rebuilt and the software re-flashed from this repository alone.
