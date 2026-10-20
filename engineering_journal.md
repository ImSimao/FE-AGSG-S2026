# AGSG — Engineering Journal
## WRO Future Engineers 2026 — Self-Driving Car

**Team:** AGSG (Portugal) — Joana Silva, Simão Freire
**Coach:** Tiago Severino

---

## Table of contents

1. [Introduction](#1-introduction)
2. [Mobility & Mechanical Design](#2-mobility--mechanical-design)
   - [2.1 Chassis and dimensions](#21-chassis-and-dimensions)
   - [2.2 Drive train](#22-drive-train)
   - [2.3 Steering](#23-steering)
   - [2.4 Motor selection and speed reasoning](#24-motor-selection-and-speed-reasoning)
3. [Power & Sensor Architecture](#3-power--sensor-architecture)
   - [3.1 Power distribution](#31-power-distribution)
   - [3.2 Power budget](#32-power-budget)
   - [3.3 Sensors](#33-sensors--selection-and-placement)
   - [3.4 Calibration](#34-calibration)
   - [3.5 Gyro evaluation](#35-gyro-evaluation)
4. [Software Architecture & Obstacle Strategy](#4-software-architecture--obstacle-strategy)
   - [4.1 Overview](#41-overview--three-boards)
   - [4.2 Modules](#42-main-controller--modules)
   - [4.3 Concurrency](#43-concurrency--two-cores)
   - [4.4 State machine](#44-state-machine-obstacle-challenge)
   - [4.5 Field model](#45-field-model)
   - [4.6 Navigation and control](#46-navigation-and-control)
   - [4.7 Traffic-sign detection and passing](#47-traffic-sign-detection-and-passing)
   - [4.8 Noise handling](#48-noise-handling)
   - [4.9 Testing, tuning and validation](#49-testing-tuning-and-validation)
   - [4.10 Edge cases](#410-edge-cases)
5. [Systems Thinking & Engineering Decisions](#5-systems-thinking--engineering-decisions)
   - [5.1 Constraints](#51-constraints)
   - [5.2 Key trade-offs and decisions](#52-key-trade-offs-and-decisions)
   - [5.3 Iteration history](#53-iteration-history)
   - [5.4 Risk & failure analysis](#54-risk--failure-analysis)
6. [Reproducibility & GitHub](#6-reproducibility--github)
7. [To be completed](#7-to-be-completed)

---

## 1. Introduction

This journal documents the engineering process behind our self-driving vehicle for the WRO Future Engineers 2026 season. It explains not only *what* we built, but *why* we made each decision, the constraints we worked under, the trade-offs we faced, and the iterations that led to the final design.

The vehicle is a compact **Ackermann-steering car** with a single rear **DC motor**, a front **servo**, and a sensor set composed of an **IMU**, four **ultrasonic sensors**, and a **camera**. It navigates the field autonomously, follows lanes, obeys traffic signs, avoids obstacles and parks itself.

The control software runs on **three programmable boards**: a Raspberry Pi Pico 2 (main controller), a Raspberry Pi Pico 1 (sonar controller) and an OpenMV H7 (camera), all programmed in **MicroPython**.

---

## 2. Mobility & Mechanical Design

### 2.1 Chassis and dimensions

The vehicle measures **287 mm (length) × 61 mm (width) × 170 mm (height)**. We aimed for the smallest practical footprint so the robot manoeuvres easily between the walls and obstacles.

The front of the body is a raised section that stays **above** the height of the field walls (100 mm), so it passes over low obstacles without touching them. The lower part of the chassis — the part that interacts with the field elements — is only **127 mm** long. This keeps the vehicle short at wall level while the raised section provides clearance over low obstacles.

### 2.2 Drive train

The drivetrain is built from:

| Element | Choice | Reasoning |
|---------|--------|-----------|
| Drive motor | **N20 DC motor, 1000 RPM @ 12 V, with quadrature encoder** | compact, geared, encoder for odometry |
| Gear reduction | **3:5** (wheel turns 3/5 of the motor speed) | increases torque at the wheel while keeping the motor in its efficient range |
| Differential | **Metal RC differential (Wltoys 144001 type)** | lets the two rear wheels rotate at different speeds through corners |
| Rear axle | driven by the differential | — |

### 2.3 Steering

The front wheels use **Ackermann steering geometry**, driven by a single **Savox SH-0255 MG servo**. Ackermann geometry makes the inner wheel turn more sharply than the outer wheel in a corner, so the wheels roll without slipping — this is important because our odometry assumes the wheels do not slip.

### 2.4 Motor selection and speed reasoning

**Requirement:** a small motor with an encoder, enough torque, and a maximum speed of about **80 cm/s**.

We compared the N20 motor family (12 V) at different gearbox ratios:

| Reduction ratio | No-load speed (RPM) | Rated speed (RPM) | Max torque (kg·cm) | Wheel speed at rated (cm/s) |
|----------------:|--------------------:|------------------:|-------------------:|----------------------------:|
| 10 (3000 RPM) | 3000 | 2300 | 0.20 | ~231 |
| 30 (1000 RPM) | 1000 | 770 | 0.48 | ~77 |
| 50 (600 RPM) | 600 | 460 | 0.80 | ~46 |

The **3000 RPM** version is far too fast (and has the lowest torque), while the **600 RPM** version is too slow. The **1000 RPM** version hits the ~80 cm/s target (≈77 cm/s at rated speed) with enough torque reserve.

With the 3:5 external reduction, the wheel speed is:

```
wheel RPM = motor RPM × (3/5)
wheel speed (cm/s) = wheel RPM × π × 3.2 cm / 60
```

At rated speed the 1000 RPM motor gives ~77 cm/s at the wheel; no-load gives ~100 cm/s. We run at a target speed of **45–60 cm/s**, below the rated speed, so the motor keeps torque reserve for acceleration and the wheels do not slip.

**Measured currents (N20 1000 RPM):** no-load 75 mA, load 0.3 A, stall 0.9 A.

---

## 3. Power & Sensor Architecture

### 3.1 Power distribution

The battery is **3× 18650 Li-ion cells in series (11.1 V nominal, 1800 mAh)**. We split the power into three rails so that the noisy motor does not disturb the sensitive electronics:

| Rail | Regulator | Voltage | Feeds |
|------|-----------|---------|-------|
| Motor | LM2596 (buck) | 11 V | TB6612FNG → motor |
| Servo | L7805CV #1 | 5 V | Savox servo |
| Logic/sensors | L7805CV #2 | 5 V | Pico 2, Pico 1, BNO085, HC-06, SRF04 ×4, OpenMV |

Separating the motor rail from the logic rail was a direct consequence of a problem we hit early on: motor switching noise was coupling into the encoder signal (see §4 and §5). The full wiring diagram is in `schemes/`.

### 3.2 Power budget

> **[TBD — measure]** Current draw of each rail (motor stall/running, servo peak, logic+sensors). The cells are 1800 mAh; we will add the measured figures and the resulting endurance estimate here.

### 3.3 Sensors — selection and placement

| Sensor | Model | Role | Why this one |
|--------|-------|------|--------------|
| IMU | **BNO085** | heading | fused **game rotation vector** without a magnetometer — immune to the magnetic fields that made last year's CMPS12 compass unusable (see §3.5) |
| Distance | **4× SRF04 ultrasonic** (front, rear, left, right) | wall/obstacle distance | cheap, simple, sufficient range for the field |
| Camera | **OpenMV H7 R1** | traffic-sign colour detection | boots instantly, onboard colour detection, streams detections over UART (replaced a Raspberry Pi Zero + Pi Camera, whose boot was too slow) |
| Encoder | **quadrature encoder on the N20 motor** | odometry (distance) | provides the distance travelled |

**Placement:** the four sonars point forward, backward, left and right; the camera faces forward to see the traffic signs; the IMU is mounted on the chassis so its axes align with the vehicle.

### 3.4 Calibration

- **Encoder** — the quadrature signal is debounced in software (a **dynamic debounce** whose window scales with speed) because the motor produces electrical noise (see §4).
- **IMU** — uses the game rotation vector, so no compass calibration is required; the yaw reference is set at the start of a run.
- **Sonar** — the four sensors are measured **sequentially** to avoid cross-talk between them (see §4).

### 3.5 Gyro evaluation

We compared two heading sensors:

| Sensor | Magnetometer | Drift per lap | Notes |
|--------|:------------:|--------------:|-------|
| CMPS12 | yes | ~0° | but on tracks with magnetic fields the heading can jump by ~40° within a single lap |
| BNO085 (game rotation vector) | no | ~1° | small, consistent drift; unaffected by magnetic fields |

The CMPS12 reads ~0° drift in a clean environment, but the competition track can contain magnetic fields that cause errors of up to ~40° in a single lap — unusable for navigation. The BNO085 drifts about 1° per lap, but that drift is small, consistent and immune to magnetic fields; the remaining drift is corrected by the sonar-based odometry corrections (§4.6).

---

## 4. Software Architecture & Obstacle Strategy

### 4.1 Overview — three boards

| Board | Firmware | Responsibility |
|-------|----------|----------------|
| Raspberry Pi Pico 2 (main) | `src/` (MicroPython) | navigation, control, sensor fusion |
| Raspberry Pi Pico 1 (sonar) | `src/SonarSlave/` | reads the 4 sonars, streams distances over UART |
| OpenMV H7 | `src/Camera/` | detects red/green traffic signs, streams detections over UART |

### 4.2 Main controller — modules

| Module | Responsibility |
|--------|----------------|
| `main.py` | entry point |
| `init.py` | hardware init (300 MHz clock, UARTs, camera, servo); starts the background thread |
| `obstacle_challenge.py` | the challenge **state machine** |
| `move_coordinates.py` | navigation primitives (move to a point, rotate) using odometry + PID |
| `background.py` | real-time control loop (sensors, odometry, cruise control, telemetry) |
| `state.py` | shared robot state (odometry, lane, direction) |
| `field.py` | field model (lanes, zones, traffic signals) |
| `camera.py` | parses OpenMV traffic-sign detections |
| `distance.py` | parses sonar distances (framed protocol) |
| `encoder.py` | quadrature encoder with dynamic debounce |
| `motor.py` | motor PWM (20 kHz) and direction |
| `servo.py` | steering servo (50 Hz) |
| `compass.py` / `bno08x.py` | BNO085 heading |
| `pid.py` | PID controller |
| `telemetry.py` | telemetry over Bluetooth |
| `uart.py` | shared UART helper |

### 4.3 Concurrency — two cores

The main Pico runs two threads on its two cores:

- **Core 0** — the state machine (`obstacle_challenge`), which decides target speed and steering.
- **Core 1** — the `background` loop, which reads sensors at fixed rates (distance 50 Hz, camera 50 Hz, odometry 100 Hz, speed control 10 Hz) and drives the motor through the cruise controller.

We chose threads over `uasyncio` because, in our tests, the cooperative scheduler of `uasyncio` caused timing jitter in the driving loop that hurt the vehicle's behaviour. `_thread` on the RP2350 gives us two real cores, and the two threads communicate through a small shared state.

### 4.4 State machine (obstacle challenge)

The obstacle challenge is a state machine (see the flowchart in `schemes/`):

1. **Determine driving direction** — at the start, the robot compares the left and right sonar readings and picks clockwise or counter-clockwise accordingly.
2. **Park out** — it leaves the starting parking lot and aligns with the outer lane.
3. **Lane following — 3 laps** (the main loop, described below).
4. **Park** — it returns to the parking lot and parks parallel to the wall, using the sonars to align.

The state machine sets a target speed and a target heading; the low-level controllers (cruise control + PID) execute them.

**Lane following (the 3-lap loop):**

Each lap drives the robot around one side of the field. Inside a lap the sequence is:

1. **Detect the entry traffic sign** — the robot first locates the sign at the entrance of the corridor so that it enters the corridor on the correct side.
2. **Enter and follow the corridor**.
3. **Mid-corridor: detect the exit traffic sign** — in the middle of the corridor the robot looks for the exit sign and decides whether to switch side (from right to left, or vice-versa) or to continue straight to the end of the corridor.
4. **Exit the corridor**.

The traffic sign is an obstacle pillar: the robot **adjusts its trajectory** according to the sign's colour — a **green** pillar is passed on the **left**, a **red** pillar on the **right**.

**Lap 1 vs laps 2–3:** the first lap is used to **detect and memorise** the colour of each sign. On the next two laps the positions are already known, so the robot drives **directly** without spending time re-detecting them.

### 4.5 Field model

The field is modelled in software as **4 lanes, each divided into 3 zones, each zone having 2 possible positions (left/right)** — matching the real WRO field layout. Each position holds a traffic signal in one of four states:

- **GREEN** / **RED** — a coloured pillar has been detected.
- **UNKNOWN** — not yet confirmed.
- **EMPTY** — confirmed to have no pillar.

To mark a position **EMPTY**, the camera must keep that position inside its field of view for a threshold time (2 s). This point-of-view accumulation guarantees the robot has looked at the position long enough before concluding there is nothing there — it avoids mistaking "I have not looked yet" for "there is nothing there".

### 4.6 Navigation and control

**Odometry** — the robot's position is estimated by fusing two sources:
- **distance** from the wheel encoder: `distance = counts / (PPR / gear_ratio) × wheel_circumference`,
- **heading** from the BNO085 IMU (game rotation vector, 100 Hz fusion).

The encoder delta and the heading are integrated in the background loop at 100 Hz to update `odom_x` / `odom_y`.

**Line following** — `move_coordinates` drives the robot toward a target point. It projects the current position onto the target line and uses a **PID controller on the cross-track error** to steer, while a trapezoidal speed profile (constant speed, then deceleration near the target) controls the speed. `rotate_coordinates` / `rotate_angle` rotate the robot in place toward a point or a heading.

**Steering PID** — the cross-track error is corrected by a PID with gains `kp=0.35, ki=0.003, kd=0.5`, with the output clamped to the steering range.

**Cruise control** — an incremental speed controller (`kp=1/70`) converts the target speed into a motor PWM, with acceleration/deceleration limits (accel 1/3, decel 2/3 of full PWM per second). A guard clamps the PWM to zero when the target speed and PWM have opposite signs, preventing the motor from reversing during forward driving.

**Sonar** — the sonar controller (Pico 1) streams the four distances over UART0 using a framed protocol (header `0xA5`, four little-endian `uint16` scaled ×10, XOR checksum), so a dropped byte cannot permanently desynchronise the receiver.

**Odometry correction** — because the encoder+IMU estimate drifts, the background loop corrects the position with the sonars in two specific situations: `corrigir_corredor` (driving straight along a corridor) and `corrigir_canto` (rounding a corner). This keeps the position estimate anchored to the real walls.

### 4.7 Traffic-sign detection and passing

The OpenMV runs a colour-blob detector that finds the red and green pillars and streams one line per frame (`CAM:cx,cy,COLOR`). The main controller:

1. computes which field positions are currently inside the camera's field of view (`get_traffic_lane_inside_pov`),
2. maps each detected blob to the matching field position (`get_colour_position`),
3. records the colour in the field model (`Field.set_signal`) and accumulates the point-of-view time.

The state machine then reads the field model (`can_enter_lane`, `can_exit_lane`, `lanes_sides`) and **adjusts the trajectory** so the robot passes each pillar on the correct side (green left, red right).

### 4.8 Noise handling

The encoder signal is the most sensitive part of the system. The motor produces electrical noise that used to create spurious encoder counts. We addressed this on three levels (detailed in §5):

1. **PWM frequency** raised to **20 kHz** so switching noise is above the encoder band.
2. **Software debounce** on the encoder, made **dynamic** (scales with speed).
3. **Separation** of the motor power rail from the logic rail.

### 4.9 Testing, tuning and validation

- **Steering PID** — we learned how to tune the gains from a PID tuning video (https://youtu.be/4Y7zG48uHRo): start with proportional gain, add derivative to reduce overshoot, then integral to remove steady-state error.
- **Cruise control** — the acceleration/deceleration limits were chosen so the wheels **never slip**, because slipping corrupts the odometry.
- **Steering limits** — the maximum steering offset was set to the **largest value that does not strain the servo**.
- **Validation** — we validate with long runs (e.g. **10 consecutive laps**) and require them to never fail; we drive the robot to known coordinates and **measure** the reached position to confirm accuracy; and we measure the **final position error when running on odometry alone** (without the sonar corrections) to quantify the drift.

### 4.10 Edge cases

- **Traffic sign not yet seen / ambiguous** — the robot keeps searching and re-confirming the sign until it is sure of its colour, or sure that the position is empty (point-of-view threshold).
- **Invalid sonar reading (−1)** — handled by the framed protocol and by ignoring out-of-range values in the correction logic.
- **Odometry drift** — corrected by the sonar-based `corrigir_corredor` / `corrigir_canto`.
- **Rotation / movement getting stuck** — every rotation and movement has a timeout, after which the robot stops and falls back.
- **UART desynchronisation** — the sonar and camera parsers re-synchronise on the header/line and drop garbage.

---

## 5. Systems Thinking & Engineering Decisions

This section records the constraints, trade-offs and iterations that shaped the design.

### 5.1 Constraints

- **Power** — a single 11.1 V battery must supply the motor, servo and all electronics.
- **Size** — the vehicle must fit in 300×200×300 mm.
- **Processing** — the main controller is a Pico 2; heavy vision is offloaded to the OpenMV.
- **Magnetic fields** — the competition environment can contain magnetic fields, so we avoid magnetometer-based heading.

### 5.2 Key trade-offs and decisions

**1. Threads (`_thread`) vs `uasyncio`**
We first migrated the control loop to `uasyncio` for cleaner code, but the cooperative scheduler introduced timing jitter that degraded the driving. We **reverted to `_thread`** (two real cores), accepting the extra care needed for shared state in exchange for deterministic timing.

**2. Sonar measurement — concurrent vs sequential**
Measuring all four sonars at once caused **cross-talk** (echoes from one sensor being read by another). We switched to **sequential measurement**, accepting a lower update rate in exchange for clean readings.

**3. Encoder debounce — fixed vs dynamic**
A fixed debounce could not handle both fast and slow speeds: at low speed the real edges are sparse and the noise dominates. We implemented a **dynamic debounce** whose window scales with the current speed, and we also raised the **PWM to 20 kHz** to move the noise source above the encoder band. These two changes together fixed the encoder over-counting.

**4. Cruise control reversal guard**
The incremental cruise controller could drive the motor PWM negative during deceleration, reversing the motor. We added a **guard** that clamps the PWM to zero when the target speed and PWM have opposite signs.

**5. Motor and gearing choice**
We chose a **1000 RPM N20 with a 3:5 reduction** so the wheel turns at 600 RPM — enough top speed (~100 cm/s theoretical) while keeping torque reserve, and we run well below the maximum (45–60 cm/s) for control stability.

**6. Raised front section**
The raised front passes over low obstacles without touching them, keeping the lower chassis short (127 mm). This is a mechanical decision, not a software one, and it is described in §2.1.

**7. Heading sensor — CMPS12 → BNO085**
Last year we used a CMPS12 compass module, but the magnetic fields in the competition environment made the heading unreliable. We switched to the **BNO085**, which uses a fused game rotation vector without a magnetometer, giving a stable heading that is unaffected by magnetic interference.

**8. Vision — Raspberry Pi Zero + Pi Camera → OpenMV**
Last year we used a Raspberry Pi Zero with a Pi Camera, but its Linux boot time before each run was too long. We switched to the **OpenMV H7**, which boots instantly and runs the colour detection onboard, streaming only the detections over UART.

**9. Distance sensing — ultrasonic vs LiDAR/IR**
We chose **ultrasonic (SRF04)** over LiDAR or IR distance sensors because we cannot know the reflectivity and colour of the competition walls in advance — LiDAR and IR depend on those surface properties, so their readings can vary with the wall. Ultrasonic is less accurate, but its measurements are more **consistent across different wall conditions**, which matters more for reliable driving.

### 5.3 Iteration history

| Version | Change | Reason |
|---------|--------|--------|
| v1 | basic drive + encoder | first running prototype |
| v2 | `uasyncio` control loop | cleaner code |
| v3 | revert to `_thread` | `uasyncio` caused driving jitter |
| v4 | sequential sonar | fix cross-talk |
| v5 | 20 kHz PWM + dynamic debounce | fix encoder over-counting |
| v6 | cruise-control reversal guard | prevent unintended reverse |

> **[TBD — confirm]** We will align these version numbers with our actual git history and photos.

### 5.4 Risk & failure analysis

| Failure mode | Effect | Mitigation |
|--------------|--------|------------|
| Encoder noise (motor) | wrong odometry | 20 kHz PWM, dynamic debounce, separated power rails |
| Sonar cross-talk | wrong distances | sequential measurement |
| Motor reversing on deceleration | wrong direction | cruise-control guard |
| Traffic-sign misdetection | wrong passing side | colour thresholds + region filtering on OpenMV |
| UART desync (sonar) | corrupted distances | framed protocol with header + checksum |

---

## 6. Reproducibility & GitHub

The repository is structured so another team can rebuild the vehicle:

- `README.md` — system description (English, ≥5000 chars), module responsibilities and build/upload steps.
- `models/` — CAD and 3D-printable files.
- `schemes/` — wiring diagrams (communication and power).
- `src/` — all control software (main, sonar slave, camera firmware).
- `t-photos/`, `v-photos/`, `video/` — team/vehicle photos and driving video.

The commit history shows the development over time, and the README plus code comments document how to flash each board.

---

## 7. To be completed (measurements & artefacts)

The following items need real data from our tests/photos before final submission:

1. Motor stall/running current and measured top speed (§2.4).
2. Power budget figures (§3.2).
3. Version/iteration history aligned with the git log (§5.3).
4. Team photos (`t-photos/`), vehicle photos (`v-photos/`), driving video (`video/`).
5. CAD files in `models/`.
