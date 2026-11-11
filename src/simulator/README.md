# AGSG simulator — firmware in the loop

A browser UI owns the world (arena, walls, traffic signs, parking bay, Ackermann
physics); the Python process is the brain. By default the brain is the **real
MicroPython firmware** in [`../robot/`](../robot/), run under CPython through a
small hardware shim that emulates the sonars, encoder, IMU and camera. A commit to
the firmware changes the next simulated round with no porting step — the same
`obstacle_challenge()` state machine drives the virtual car and the real one.

```
browser (physics + scoring)  <--ws-->  simulator/  <--json lines-->  robot/
  pose, sonar, signs, encoder           controller                 MicroPython code
```

## Run it

```bash
cd src                            # so `python -m simulator` resolves the package
./simulator/start.sh              # UI on :8781, websocket on :8780
# open http://localhost:8781/
```

Ports are configurable with `UI_PORT` / `SIM_PORT` (defaults 8781 / 8780; 8765 and
8000 are usually taken by other local services). `start.sh` installs `websockets`
into a local `.venv` on first run. To run the old stand-in loop instead of the
firmware: `python -m simulator.main --controller placeholder`.

## What a round does

1. The page connects → the server resets the car into the parking bay and starts a
   fresh firmware interpreter (its state lives in module globals, so every round
   needs a new process).
2. The UI streams heading (50 Hz), encoder delta (100 Hz), the four sonar ranges
   (20 Hz) and every sign in the camera FOV (10 Hz) over the websocket.
3. The controller converts units, feeds the runner process, and turns the
   firmware's `state.target_speed` + `Servo.current_angle` back into a world
   command (`speed`, `steeringRadius = L / tan(delta)`).
4. `ui/lib/race.js` times the round, counts sections and laps, tracks sign and
   wall contact, and scores it with the official WRO 2026 table.

Buttons: **New track** (new seed), **Repeat track** (same seed, fresh firmware),
**Noise** (sensor noise + real sonar cones), **1×/2×** (simulation speed). The
seed fixes the sign layout and the start direction, so two firmware versions can
be compared on the exact same track.

## Units and frames

| quantity | world (browser) | firmware |
| --- | --- | --- |
| length | metres | cm |
| heading | degrees CCW from +x, absolute | degrees, zero at boot, sign set by `--imu-sign` |
| sonar | metres, labels Front/Rear/Left/Right | cm ×10 in a `0xA5` + XOR frame, `0xFFFF` = no echo, saturates at 250 cm |
| camera | `{lane, state(1 green / 2 red), distance, angleDeg, heightCm}` | `CAM:x,y,h,COLOR` lines, cm in the camera frame (x + right, y forward, h sign height 5/10/15) |
| encoder | metres travelled this tick | pulses injected into `encoder._count` (33.2 counts/cm: PPR 200, ratio 3/5, Ø 32 mm) |
| position | metres, origin bottom-left | absolute cm, mirrored in x when `clockwise == 1` |

The sign table in `ui/lib/traffic.js` matches `camera.py:get_traffic_lane_inside_pov()`
exactly, so the mapping between the world and the firmware's zones is derived, not
guessed.

## Knobs (all on `main.py`)

| flag | default | meaning |
| --- | --- | --- |
| `--controller` | `agsg` | `agsg` = real firmware, `placeholder` = old stand-in loop |
| `--imu-sign` | `-1` | firmware heading = sign × world heading |
| `--servo-sign` | `1` | which way a positive servo angle steers |
| `--wheelbase-mm` | `70` | AGSG wheelbase |
| `--servo-gain` / `--servo-offset-deg` | `1.0` / `0` | `delta_eff = gain × angle + offset` |
| `--actuator` | `target_speed` | or `pwm`, with `--pwm-cm-per-duty` |

UI query parameters: `?ws=ws://host:port`, `?start=centre|nose|nose_in`, `?dir=cw|ccw`.

## Bench calibration (closes the loop)

`R = L / tan(delta)` assumes the servo at 0° really points straight — it rarely
does. Drive a fixed servo angle (29°), let the car do two full circles on the
floor, read the total encoder distance and IMU heading, and compute
`R = D / theta_rad`; repeat at 15° and fit `delta_eff = k · delta + delta0`, then
run with `--servo-gain k --servo-offset-deg delta0`.

`python -m simulator.calibrate` sweeps `imu-sign`, `servo-sign`, the start pose and
the servo gain in the sim and ranks them by score, wall hits and odometry error —
its best row is the baseline until the bench numbers exist.

## Headless runs

```bash
.venv/bin/pip install playwright          # one-off; needs the chromium build
.venv/bin/python -m simulator.race_run --runs 3 --seconds 180   # → race-results.json
.venv/bin/python -m simulator.race_run --runs 2 --seconds 60 --shot /tmp/run
```

Each run prints the score breakdown as JSON and leaves a screenshot, so a firmware
change is judged by its score and its odometry error, not by hand.

## Scoring (WRO FE 2026, Obstacle Challenge, max 62)

`1.1` 1 pt per section in the right direction (24) · `1.2` 1 pt per full lap (3) ·
`1.3` 3 pts stopped in the finish section after three laps · `1.4/1.5` 2/4 pts
signs moved or not, only if fewer than three laps · `1.6/1.7` 8/10 pts after three
laps · `1.8.1` 7 pts started inside the parking lot (needs one full lap) ·
`1.8.2` 15 pts parked inside and parallel · `1.8.3` 7 pts parked partly or not
parallel. Round cap 180 s, ties broken by time.

Contact rules enforced by the world: **9.18** moving a wall zeroes the round;
**9.20** touching or knocking a sign is allowed and only costs 1.4/1.6; **9.22**
signs only count during the three official laps. Passing a sign on the wrong side
ends the round there and then (`result.signWrongSide` records why).

## Measured results

| seed | laps | sections | score | time | parking |
| --- | --- | --- | --- | --- | --- |
| 4111204647 | 3 | 24 | **62** | 112–122 s | 1.8.2 (complete and parallel) |
| 2026 | 3 | 24 | **62** | 128 s at 2× | 1.8.2 |
| «Repeat track» at 4× | 3 | 24 | **62/62/62** | 142.7 / 153.3 / 138.2 s | 1.8.2 |
| sign sweep (53 layouts) | 3 | 24 | **40 of 53** | median 133 s, max 151 s | 1.8.2 in all that finish |

Odometry vs ground truth: median **2.8 cm**, max per round **6.9 cm**. The parking
manoeuvre is the most fragile part (it lands in 1.8.2 / 1.8.3 / zero depending on
the draw).

## Tools

- `tools/sign_sweep.py` + `tools/sweep_report.py` — run one round per sign layout
  (53 layouts, 12 patterns per lane) and report which fail.
- `tools/margin_report.py` — rebuild the car body at every trajectory step and
  report the clearance to each wall and sign.
- `tools/mpy_audit.py` — flag CPython-only stdlib APIs (`os.environ`, `math.hypot`)
  that import in the simulator but crash on MicroPython, so they are caught before
  flashing. `python tools/mpy_audit.py` checks `../robot/` by default.

## Layout

```
simulator/              this package (controller, shim, runner, server)
simulator/ui/           browser UI (physics + scoring)
simulator/tools/        sign sweep, margin report, MicroPython audit
../robot/               the firmware, run unmodified
```

The simulator never edits the firmware: it runs `../robot/` byte-for-byte, so a
run against edited firmware proves nothing.
