# Changelog

All notable changes to the AGSG self-driving car are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- **Sonar slave measures sequentially** (`src/robot/SonarSlave/main.py`): the four
  SRF04 sensors are triggered one at a time, with a 3 ms guard gap after each echo
  and a wait for a stale pulse to fall before firing, instead of all four at once.
  Concurrent firing risks cross-talk, and a cross-talk spike reads as a wall a few
  centimetres away. A sweep now costs the sum of the four echoes (~50 ms at
  corridor distances, ~20 Hz; 129 ms worst case when every echo times out). The
  UART frame format is unchanged.
- **`SonarSlave/debug.py`** treats the sequential sweep as the production mode and
  keeps the concurrent pass as the comparison, so cross-talk stays measurable on
  the bench.

## [1.0.0] - 2026-11-11

The first release documented for the WRO Future Engineers 2026 season. The
vehicle drives the obstacle challenge autonomously (three laps, traffic-sign
obedience, parking) and is validated in a firmware-in-the-loop simulator.

### Added

- **Simulator** (`src/simulator/`): browser UI (physics + WRO scoring) plus a
  Python controller that runs the real MicroPython firmware under CPython
  through a hardware shim.
- **Sonar-based odometry correction** (`correct_corridor`, `correct_corner`),
  run at 40 Hz from the background task.
- **Camera traffic-sign detection** with field-of-view mapping and per-sign
  point-of-view time accumulation (a sign is only "empty" after 2 s in view).
- **Lane-following state machine** that memorises the sign colours on lap 1 and
  reuses them on laps 2–3.
- **Cruise control** with acceleration/deceleration limits, a steering-aware
  accel scale and a 500 ms reverse brake hold (no skidding).
- **PID steering** (`kp=0.35`, `ki=0.003`, `kd=0.5`).
- **MicroPython portability audit** (`tools/mpy_audit.py`) to catch CPython-only
  stdlib calls before they reach the board.
- **Bench calibration** (`bench/`, `calibrate.py`) and a headless race runner
  (`race_run.py`).
- **Engineering Journal** (`engineering_journal.md`) covering the five WRO
  documentation criteria.

### Changed

- Code reorganised into `src/robot/` (firmware) and `src/simulator/` (simulator).
- Vehicle length reduced to **200 mm** (parking-length rule measured below the
  10 cm wall) with a low front nose; weight recorded at **610 g**.
- README and journal rewritten in English and aligned with the code.

### Fixed

- Sign-convention contradiction between odometry and the camera zone table.
- `os.environ` / `math.hypot` (CPython-only) crashing on the board.
- Steering-sign bug in the simulator shim that looked like "turns into the wall".

## [0.x] - 2026-03 to 2026-10

Early development (see `git log`): initial hardware modules (BNO085, motors,
servo, sonars, encoder), odometry + PID + cruise control, OpenMV camera
firmware, and the first obstacle-challenge state machine.
