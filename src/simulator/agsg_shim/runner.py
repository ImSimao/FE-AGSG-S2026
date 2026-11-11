"""Run the vendored AGSG firmware against the shim, in its own process.

Why a process and not a thread: the firmware's state lives in module globals
with no clean reset (a second run in the same interpreter would inherit the
first run's odometry, lane index and cruise PWM), and its background loop sleeps
1 ms, which would starve the asyncio server feeding it.

Protocol, one JSON object per line.

Host -> runner:
  {"t": "h", "deg": 12.3}                 heading, degrees (IMU convention)
  {"t": "e", "cm": 0.42}                  signed travel since the last pulse
  {"t": "d", "cm": {"front": .., ...}}    four ranges in cm (-1 = no echo)
  {"t": "c", "signs": [{"state": 1, "bearingDeg": -8.0, "distance": 0.6}]}
  {"t": "clock", "scale": 2.0}            the world speed multiplier (time lapse)
  {"t": "go"}                             start the challenge state machine
  {"t": "stop"}                           tear down

Runner -> host:
  {"t": "ready"}                          firmware imported, background thread up
  {"t": "act", "v": 0.2, "r": 0.31, ...}  sampled actuator command
  {"t": "odom", "x": .., "y": .., "h": .., "cw": -1}   firmware's own estimate
  {"t": "tel", "m": "csv or log line"}    whatever the firmware wrote to UART0
  {"t": "err", "m": "..."}                uncaught exception
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys
import threading
import time

SHIM_DIR = pathlib.Path(__file__).resolve().parent
ROOT = SHIM_DIR.parent.parent
FIRMWARE_DIR = ROOT / "robot"

ACT_HZ = 50.0
ODOM_HZ = 10.0
ENC_HZ = 100.0


# ---------------------------------------------------------------- time shim


def patch_time() -> None:
  """Give the firmware a MicroPython ``time`` whose clock runs at the sim speed.

  The swap is global (``sys.modules``), but this module already bound the real
  ``time`` above, so the runner's own sleeps stay in wall-clock time.
  """
  import vtime  # noqa: E402  (same directory as hardware.py)

  sys.modules["time"] = vtime.as_module()


def _scale() -> float:
  import vtime  # noqa: E402

  return vtime.clock.scale


def out(payload: dict) -> None:
  try:
    sys.stdout.write(json.dumps(payload, separators=(",", ":")) + "\n")
    sys.stdout.flush()
  except (BrokenPipeError, ValueError):
    os._exit(0)


# ------------------------------------------------------------------- reading


def apply_scale(value) -> float:
  """Sync the firmware clock with the world. Called on every frame.

  The multiplier used to be a one-off ``{"t":"clock"}`` message sent only when
  the controller noticed a change, and the controller kept its ``_last_scale``
  across firmware restarts. A restarted firmware process therefore never heard
  about the multiplier and ran its clock at 1x while the world ran at 4x: its
  50 Hz control loop became 12.5 Hz of simulated time, its ``sleep(1)`` held for
  four simulated seconds, and every velocity it measured was a quarter of the
  truth. Carrying the value on the frames (which are already flowing) makes the
  firmware's clock self-correcting instead of remembering state.
  """
  import vtime  # noqa: F401  (same directory as hardware.py)

  try:
    scale = float(value)
  except (TypeError, ValueError):
    return vtime.clock.scale
  if scale <= 0:
    return vtime.clock.scale
  changed = abs(scale - vtime.clock.scale) > 1e-6
  vtime.set_scale(scale)
  if changed:
    out({"t": "stderr", "m": "[shim] relogio do firmware a %.1fx" % scale})
  return vtime.clock.scale


def reader_loop(hal) -> None:
  import vtime  # noqa: E402  (same directory as hardware.py)
  for line in sys.stdin:
    line = line.strip()
    if not line:
      continue
    try:
      msg = json.loads(line)
    except json.JSONDecodeError:
      continue
    kind = msg.get("t")
    # Every frame carries the multiplier, so a firmware that (re)starts mid
    # round picks it up on its very first sample.
    if "s" in msg or "scale" in msg:
      apply_scale(msg.get("s", msg.get("scale")))
    # The world's clock, so the firmware's sleeps and timeouts land on the same
    # simulated tick at any multiplier. Without it the firmware's time came from
    # the wall clock multiplied by the scale, and every scheduling hiccup moved
    # the car a different distance per control step.
    if "simMs" in msg and msg["simMs"] is not None:
      vtime.clock.sync_to_sim(msg["simMs"])
    if kind == "h":
      hal.set_heading(msg.get("deg", 0.0))
    elif kind == "e":
      hal.add_travel_cm(msg.get("cm", 0.0))
    elif kind == "d":
      hal.set_distances_cm(msg.get("cm") or {})
    elif kind == "c":
      hal.set_signs(msg.get("signs") or [])
    elif kind == "clock":
      apply_scale(msg.get("scale"))
    elif kind == "go":
      hal.started = True
    elif kind == "stop":
      hal.stopping = True
      break
  hal.stopping = True


# -------------------------------------------------------------------- output


def encoder_loop(hal, encoder_module) -> None:
  """Inject pulses into the firmware's own counter (never fake distance_cm())."""
  counts_per_cm = (encoder_module.PPR / encoder_module.RATIO) / (
    math.pi * encoder_module.WHEEL_DIAMETER
  )
  residual = 0.0
  period = 1.0 / ENC_HZ
  while not hal.stopping:
    cm = hal.take_travel_cm()
    if cm:
      exact = cm * counts_per_cm + residual
      whole = int(round(exact))
      residual = exact - whole
      if whole:
        encoder_module._count += whole  # the firmware's own counter
    # The hardware side lives in sim time too: at 2x the world advances twice as
    # fast, so the sensor/actuator loops must tick twice as often per real second.
    time.sleep(period / max(1e-6, _scale()))


# WRO_TEL_ECHO=<substring> mirrors the firmware's own telemetry lines that
# contain <substring> to stderr, which the controller puts in the UI log and the
# exported round keeps. Without it those lines are parsed and dropped, so a
# firmware that logs its decisions is silent. Never echo unfiltered: the line
# rate is 20 Hz and it starves the websocket.
_ECHO_TEL = os.environ.get("WRO_TEL_ECHO") or None


def telemetry_loop(hal) -> None:
  while not hal.stopping:
    if hal.tx:
      pending = hal.tx[:]
      del hal.tx[:]
      for line in pending:
        out({"t": "tel", "m": line})
        if _ECHO_TEL and _ECHO_TEL in line:
          out({"t": "stderr", "m": "[tel] " + line})
    time.sleep(0.02 / max(1e-6, _scale()))


_state_module = None


def state_module_ref():
  return _state_module


def camera_summary(camera_module) -> dict:
  """How many blobs the firmware parsed, and how many zones it considers in view."""
  if camera_module is None:
    return {}
  try:
    cam = camera_module.Camera
    pov = []
    try:
      pov = camera_module.get_traffic_lane_inside_pov()
    except Exception:  # noqa: BLE001
      pov = []
    st = state_module_ref()
    sonars = None
    try:
      import background as background_module

      sonars = [round(float(v), 1) for v in background_module.get_odom_side_sonar()]
    except Exception:  # noqa: BLE001
      sonars = None
    zones = ""
    sides = None
    gates = None
    povms = None
    try:
      import field as field_module

      lane = field_module.Field.LANES[int(getattr(st, "relative_lane", 0)) % 4]
      symbols = {"RED": "R", "GREEN": "G", "UNKNOWN": ".", "EMPTY": "-"}
      zones = "|".join(
        "".join(symbols.get(str(v), "?") for v in zone) for zone in lane.zones
      )
      gates = [
        bool(lane.can_register_signal_in_zone(lane.zones[0:2])),
        bool(lane.can_register_signal_in_zone(lane.zones)),
        bool(lane.can_register_signal_in_zone(lane.zones[1:3])),
      ]
      povms = [int(v) for v in lane.zone_pov_ms[0]]
      sides = [int(v) for v in field_module.Field.lanes_sides()]
    except Exception as exc:  # noqa: BLE001
      zones = "?%s" % type(exc).__name__
    return {
      "laps": int(getattr(st, "lap", -1)) if st else None,
      "zones": zones,
      "sides": sides,
      "son": sonars,
      "gates": gates,
      "povms": povms,
      "blobs": len(getattr(cam, "_last_blobs", []) or []),
      "pov": len(pov),
      "povItems": [int(p.get("pos", -1)) for p in pov],
      "rel": [round(float(v), 1) for v in (st.get_relative_odom if st else (0, 0))],
      "hdg": round(float(getattr(st, "compass_angle_relative", 0.0)), 1) if st else None,
      "lane": int(getattr(st, "current_lane", 0)) if st else None,
      "cw": int(getattr(st, "clockwise", 0)) if st else None,
    }
  except Exception:  # noqa: BLE001
    return {}


def actuator_loop(hal, state_module, servo_module, distance_module=None, camera_module=None) -> None:
  period = 1.0 / ACT_HZ
  odom_every = max(1, int(ACT_HZ / ODOM_HZ))
  i = 0
  while not hal.stopping:
    servo_angle = float(getattr(servo_module.Servo, "current_angle", 0.0))
    target_speed = float(getattr(state_module.state, "target_speed", 0.0))
    motor = hal.motor_state()
    sonar = None
    if distance_module is not None:
      try:
        sonar = {k: round(float(v), 2) for k, v in distance_module.Distance._last_sensor_data.items()}
      except Exception:  # noqa: BLE001
        sonar = None
    out(
      {
        "t": "act",
        "v": target_speed / 100.0,  # cm/s -> m/s
        "a": servo_angle,
        "duty": round(motor["duty"], 4),
        "dir": motor["direction"],
        "brake": motor["brake"],
        "sd": sonar,  # what the firmware decoded from the sonar frame
        "cb": camera_summary(camera_module),  # what the camera parser produced
      }
    )
    if i % odom_every == 0:
      st = state_module.state
      out(
        {
          "t": "odom",
          "x": round(float(st.odom_x), 2),
          "y": round(float(st.odom_y), 2),
          "h": round(float(st.compass_angle_relative), 2),
          "cw": int(st.clockwise),
          "lane": int(st.current_lane),
        }
      )
    i += 1
    # The hardware side lives in sim time too: at 2x the world advances twice as
    # fast, so the sensor/actuator loops must tick twice as often per real second.
    time.sleep(period / max(1e-6, _scale()))


# ---------------------------------------------------------------------- main


def main() -> None:
  patch_time()
  sys.path.insert(0, str(SHIM_DIR))
  sys.path.insert(1, str(FIRMWARE_DIR))

  import hardware  # noqa: E402  (after sys.path setup, on purpose)
  import vtime  # noqa: E402

  # Tell the controller when the firmware blocks in a sleep: in lock-step that is
  # the signal to advance the world by exactly one physics step.
  vtime.clock.on_wait = lambda waiting: out({"t": "wait", "w": bool(waiting)})

  hal = hardware.hal
  threading.Thread(target=reader_loop, args=(hal,), daemon=True).start()

  # Importing the firmware runs its hardware init exactly as on the board:
  # init.py opens both UARTs, starts the background control thread and centres
  # the servo.
  import init  # noqa: E402,F401
  import main as firmware_main  # noqa: E402
  import state as state_module  # noqa: E402
  global _state_module
  _state_module = state_module.state
  import servo as servo_module  # noqa: E402
  import encoder as encoder_module  # noqa: E402
  import distance as distance_module  # noqa: E402
  import camera as camera_module  # noqa: E402

  threading.Thread(target=telemetry_loop, args=(hal,), daemon=True).start()
  threading.Thread(target=encoder_loop, args=(hal, encoder_module), daemon=True).start()
  threading.Thread(
    target=actuator_loop,
    args=(hal, state_module, servo_module, distance_module, camera_module),
    daemon=True
  ).start()

  out({"t": "ready"})

  # Wait for the world to be live (sensors valid, UI connected) before the
  # state machine starts, then run it for real: main() blocks forever.
  while not hal.started and not hal.stopping:
    time.sleep(0.02 / max(1e-6, _scale()))

  if hal.stopping:
    os._exit(0)

  try:
    firmware_main.main()
  except BaseException as exc:  # noqa: BLE001 - report anything the firmware raises
    out({"t": "err", "m": "{}: {}".format(type(exc).__name__, exc)})
    raise
  finally:
    os._exit(0)


if __name__ == "__main__":
  main()
