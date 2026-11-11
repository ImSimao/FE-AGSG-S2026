# AGSG firmware inside the simulator — findings

Everything below was observed by running the **unmodified** firmware from
`ImSimao/FE-AGSG-S2026` (vendored in `src/robot/`) against this world.
Nothing in `src/robot/` was edited: a run against edited firmware proves nothing.

## What works

* `src/robot/` imports and boots as on the board (`init.py` opens both UARTs, starts the
  background thread on core 1, centres the servo).
* Sonar packets, camera frames, IMU heading and encoder pulses reach the firmware
  through its own parsers (`distance.py` resync + checksum, `camera.py`
  `CAM:cx,cy,COLOR`, `encoder.py` pulse counter).
* Actuator output is read back as `state.target_speed` (cm/s) and
  `Servo.current_angle`, turned into world speed and a geometric steering radius
  `R = L / tan(delta)` with `L = 70 mm`.
* **Odometry fidelity: 2–8 cm** between the firmware's own `state.odom` and the
  world's true pose over a 8 m run (shown live as the "Erro odometria" tile).
  That is the number that says the units, signs and frames are right.
* A full WRO FE 2026 round is timed and scored (see `src/simulator/ui/lib/race.js`).

## Issue 1 — the clockwise start drives into the wall (blocker for half the draws)

With the car parked facing **west** in the bottom bay (outer wall on its left, so
the firmware computes `clockwise = 1`), the boot sequence turns left and drives
into the bottom wall within 0.15–3 s:

| start pose | result |
| --- | --- |
| `start=nose` (nose 1.75 cm from the wall ahead) | moves 0.16 m, wall hit at 1.0 s |
| `start=nose_in` (nose against the opposite wall) | wall hit at 0.15 s |

The same car started facing **east** (`clockwise = -1`) drives 6 of the 8 sections
and 8.4 m without touching anything.

Why: the two heading corrections do not compose.

* `get_angle_to_rotate()` negates the compass angle **only** for `clockwise == -1`.
* `rotate_coordinates()`/`rotate_angle()` use `servo_angle = MAX_STEERING_OFFSET * clockwise`.

With a *fixed* IMU sign, one of the two branches ends up commanding the wrong
direction: the error term and the steering term have the same sign relation in
both branches, but the compass only gets mirrored in one. In the simulator this
shows up as a divergent rotation; on the real car it is invisible if the IMU
happens to be mounted with the other sign, or if the team only ever starts in one
direction.

Suggested check on the bench: boot the car in the bay facing each direction and
run `rotate_angle(0)` — it should settle, not spin. Then swap `state.clockwise`'s
meaning or mirror `Servo.set_angle` consistently.

Reproduce: `?dir=cw` in the UI, or `--starts nose --imus -1 --servos 1` with the
direction forced (the calibration harness does this).

## Issue 2 — the camera assigns a colour to the first zone in view

`camera.py:get_colour_position()` ends with

```python
for position in possible_traffic_positions:
    Field.set_signal(position["pos"], position["side"], signal)
    return
```

The blob's `cx, cy` are only used for the `hypot(dx, dy) <= 65` gate; the angle
test is commented out. So when two signs are in the POV at once, the colour is
written to whichever zone is first in the list — the firmware cannot tell them
apart. The simulator emits *every* sign in the FOV (rather than the nearest one)
so the same ambiguity appears here instead of only at the competition.

## Issue 3 — the 3 s "search" pause looks like a finished round

`obstacle_challenge()` waits up to 3000 ms on `Field.can_exit_lane()` while
standing still. The scoring module therefore uses a 5.5 s stop window before
calling a round finished — otherwise every colour search would end the run.
The WRO judge has the same problem and uses the 180 s cap.

## Issue 4 — boot anchors are hard-coded to one bay

`obstacle_challenge()` opens with

```python
state.set_relative_odom(100 + 1.75, distance_left)      # clockwise
state.set_relative_odom(200 - distance_front, distance_right)  # otherwise
```

and parking uses `parking_gap = 45`, `parking_y = 29`, `parking_x_offset = 5`.
Those numbers only describe a bay whose walls sit at 100/145 cm (or 155/200 cm)
with the car 1.75 cm from one wall. Any other bay position (other field, other
draw, the simulator's bay) puts the whole plan a few centimetres off from the
first frame. Deriving the anchor from the sonar reading instead of a constant
would make the boot independent of the bay.

## Issue 5 — parking manoeuvre is marginal in this world

The final sequence (turn into the bay, front sonar to 18 cm, reverse until the
rear sonar reads 7.5 cm) is the part most sensitive to the world model: it needs
the two magenta walls and the 45 cm bay exactly where the firmware expects. In the
sim it currently completes "1.8.3 parked partly" only when the car happens to stop
inside the rectangle. Worth re-checking once Issue 1 is fixed, because a
clockwise round parks at 105 cm, which is byte-for-byte the constant in the code.

## Not modelled (on purpose)

* wheel slip and tyre scrub — a real car at 70 cm/s cannot follow a 12.6 cm radius
  arc, the simulator lets it. This makes the sim slightly *optimistic* in tight
  corners.
* sonar cross-talk, echo dropouts, and the 250 cm saturation are clamped but not
  probabilistic.
* the HB-06 Bluetooth link (the firmware's telemetry is captured on the shim UART
  and counted, not shown).

## Verified: the sonar chain is faithful

Measured with the car parked in the bay (world value vs what the firmware decoded
from the 0xA5 frame):

```
world    F 18.0  R 15.7  L 79.5  R 11.2 cm
firmware F 17.9  R 15.5  L 82.7  R 13.4 cm
```

and, with the car stationary, the two agree to **0.0 cm** on every sample. While
driving the gap is 1-4 cm, which is just the sampling skew (the world sample is
taken at send time, the firmware decodes a frame 20-40 ms older at 70 cm/s).

So the framing, the checksum resync, the cm/×10 scaling, the label order and the
sat/timeout sentinel are all correct. Sensor/protocol hypotheses are closed: what
is left is the firmware's own navigation maths.

## The bug was in the shim: the steering sign was being collapsed

`_radius_from()` did `max(min_radius, abs(radius)) * servo_sign`, which threw away
the sign of `tan(delta)` and then multiplied by a constant. Every steering command
therefore came out with the same sign: the car held a constant lock and orbited,
which looked exactly like "the firmware turns left into the wall". Fixed by keeping
the sign (`copysign`) before applying `servo_sign`.

This invalidated every earlier comparison between sign pairs: with one direction
removed, the two pairs were indistinguishable. Re-measured after the fix, both
survive a 40 s round (1-3 of 24 sections, 6-27 cm odometry error, heavy wall
scraping), so the pair cannot be chosen from the simulator alone - it needs the
bench test on the car.

## The remaining contradiction: odometry vs camera zones

The camera chain is complete (frames are delivered, the firmware parses 1-3 blobs
per frame), but no colour reaches `Field`: `get_traffic_lane_inside_pov()` returns
an empty list, so `get_colour_position()` has no zone to write to.

Measured over 14 samples at the same track, 4 sign pairings:

| imu | servo | blobs seen | samples with a zone in view |
| --- | --- | --- | --- |
| +1 | +1 | 0 | 0/14 |
| +1 | -1 | 2 | 8/14 |
| -1 | +1 | 0 | 13/14 (from odometry alone) |
| -1 | -1 | 3 | 0/14 |

There is no pairing that gives both a working camera POV **and** a sane odometry.
The reason is in `background.py`:

```python
state.odom_x += delta_cm * cos(theta) * clockwise * -1
state.odom_y += delta_cm * sin(theta) * clockwise
```

The two components carry opposite `clockwise` factors, which mirrors y instead of
rotating: consistent only if `compass_angle` is clockwise-positive. But the traffic
coordinate table in `camera.py` (`[100,60]`, `[150,60]`, …) is expressed in the
world frame, which is counter-clockwise-positive. Whichever sign is fed to the IMU,
one of the two breaks:

- counter-clockwise compass -> odometry is right (2-25 cm) and the POV is empty;
- clockwise compass -> the POV finds zones and the odometry runs away
  (`odom_y` reached -57 cm, impossible on a 300 cm field).

Fix on the firmware side: make the odometry use one sign convention end to end
(either both components with the same `clockwise` factor, or flip the compass once
when it is read) and keep the traffic table in that same frame. Then one IMU sign
will satisfy both.

## Integrated upstream 74893b4 (2026-11-03)

`src/robot/sync_firmware.sh` pulled `74893b4 Update odometry handling and adjust
speed limits`. Their own changes, and what they change for the simulator:

- `ADJUST_ODOM_INTERVAL_MS` 200 -> 25: the sonar corrections now run at 40 Hz
  instead of 5 Hz, so the odometry no longer drifts for 200 ms between fixes.
- New `update_odom()` choke point with a 20 cm rejection gate. This is the same
  idea as the outlier gate in our patch, and it is the right place to add the
  blend, so the patch is now four lines there instead of nine scattered calls.
- `correct_corner()` now accepts both +90 and -90 (the old test only ever matched
  one of the two corner directions) and rejects readings within 3 cm.
- `move_coordinates` max speed 70 -> 50 cm/s: on the real car the motor was
  saturating, so the commanded speed and the achieved speed disagreed.

Re-measured with the blend on top of their version (same track, same seed, 45 s,
ghost error against the simulator's truth):

| | mean | p90 | max |
| --- | --- | --- | --- |
| upstream 74893b4 | 4.13 | 6.48 | 10.21 |
| + blend (K=0.2) | 3.47 | 5.88 | 7.03 |

So the two changes are complementary rather than redundant: their gate rejects
the wild readings, the blend stops the remaining small corrections from throwing
away the encoder/IMU estimate.

### One thing for the team

`update_odom()` writes two `Telemetry.log` lines per correction, and it now runs
at 40 Hz. That is ~80 long lines per second on top of a telemetry stream designed
for 20 Hz. On the bench with the HC-06 that will saturate the link and starve the
pose/heartbeat lines. Worth putting behind a debug flag before the next round.

Added 2026-11-10: `Telemetry.ENABLED` in `src/robot/telemetry.py` is that
flag — one switch for the CSV frame and every log line, read at import and
checked on each write, so it also works as a runtime toggle. It starts **OFF on
the car and ON in the simulator**, decided by `sys.implementation.name`
(`"micropython"` vs `"cpython"`); `WRO_TELEMETRY` overrides either way wherever
an environment exists. Worth knowing when tuning: the UART is shared with the
sonar frame and `Distance.update()` runs in the same thread as `Telemetry.send()`,
so a blocking write delays sensor reads too, not just the log. Measured in the
shim over 3 s of simulated time: 58 `tel` frames with it on, none with it off,
while `act`/`odom` stayed identical (141 vs 144).

## Issue 6 — the board's stdlib is a subset, and the simulator cannot see it (2026-11-10)

The simulator runs `src/robot/` on CPython; the car runs MicroPython, whose
stdlib is documented as "a subset of the corresponding CPython module". A name
CPython has and MicroPython does not imports fine, runs fine in the sim, and
raises only on the board. Two rounds in a row died that way:

| call | traceback | where it stopped |
| --- | --- | --- |
| `os.environ.get("WRO_TEST_PARKING")` | `AttributeError: 'module' object has no attribute 'environ'` | at boot, before the car moved — an embedded port has no process environment |
| `math.hypot(dx, dy)` | `AttributeError: 'module' object has no attribute 'hypot'` | the first `rotate_coordinates()` of the clockwise exit |

The shape of the message is the tell: MicroPython's `AttributeError` names the
type, not the module (`'module' object has no attribute ...`), so a CPython-shaped
message means the simulator raised it and this class is not the cause.

Both fixed in place — `math.sqrt(dx*dx + dy*dy)`, and an env probe that falls
back to `False` instead of touching `os.environ` unguarded. The useful part is
`src/simulator/tools/mpy_audit.py`: it checks every stdlib name the firmware uses against the
MicroPython docs and exits non-zero, so the class costs a second instead of a
round.

```bash
.venv/bin/python src/simulator/tools/mpy_audit.py              # src/robot/, 30 files
.venv/bin/python src/simulator/tools/mpy_audit.py --self-test  # proves the rules fire
```

`src/robot/Camera/` is the OpenMV script and runs on a different MicroPython
build, so the checker skips it. A deliberate CPython-only probe (one that is
caught and falls back) is marked `# mpy-ok` on its own line.
