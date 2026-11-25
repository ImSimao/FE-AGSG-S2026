# Changelog

All notable changes to the AGSG self-driving car are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.1.0] - 2026-11-25

### Changed

- **Telemetry silent on the car**: `telemetry.py` no longer forces
  `ENABLED = True`. It is `_initial_enabled()` again, `False` on the board and
  `True` in the simulator, so a competition round writes nothing to the HC-06
  UART (rule 11.10); `WRO_TELEMETRY=1` brings the lines back for a bench session.
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

### Fixed

- **Camera: a normal 10 cm sign was read as a modified 15 cm one, and its colour
  was inverted.** The vertical fit solves distance and height together, so the
  pair scales by one factor and stays self-consistent; a pitch/height
  calibration error slides it along that family. A measured 10 cm green at
  94 cm came back as `CAM:-1.7,143.5,15.4,GREEN` (both 1.53x), 15.4 cm sits in
  the Surprise-Rule band, and the sign was written into the field as RED - a
  distance error became a colour error and the car took the wrong line. The
  camera now cross-checks the vertical fit against the range from the column's
  fixed 5 cm width: yaw can inflate that width by at most sqrt(2), so a fit
  beyond 1.45x of the width range is impossible in either direction, and it is
  re-ranged on the width with the height re-solved there.
- **`apply_surprise_rule()` inverts only on a confident 5.0 or 15.0 cm height**
  (+-0.6) instead of the 3.5-7.5 / 12.5-18.0 bands, and never when the height is
  unknown. The camera snaps the height to the three heights the rules allow, so
  5.0 and 15.0 are the signal and 10.0 is the normal sign. A missed Surprise
  costs the side of one sign; a false one costs the side of every normal sign
  that reads tall.
- **Mounting constants refitted**: `CAM_HEIGHT_CM` 10.18 -> 10.95 and
  `CAM_PITCH_DEG` 8.3 -> 11.0, from the measured pose above. Provisional - one
  pose pins two unknowns exactly, so a second and a third distance at the bench
  have to confirm it (`--fit` below).

### Added

- `src/simulator/tools/camera_fit.py`: `--self-test` replays the field-log pose
  through the camera script's own geometry, `--sweep` shows what the pipeline
  reports when the mounting is off, and `--fit` does least squares on
  `CAM_HEIGHT_CM` / `CAM_PITCH_DEG` from a table of measured poses.

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
