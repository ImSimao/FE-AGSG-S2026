"""AGSG firmware controller: replaces the placeholder loop with the real brain.

The world (browser) publishes sensor streams over the websocket; this controller
converts them into the units the firmware expects, feeds the runner process, and
turns the firmware's actuator output back into a speed/steering command for the
world.

Every conversion is a named knob (see ``Config``) because they are exactly the
places where firmware-in-the-loop integrations go wrong silently:

* ``imu_sign``  - the BNO085 heading sign relative to the world's CCW convention
* ``servo_sign``- which way a positive servo angle steers
* ``wheelbase_m`` / ``servo_gain`` / ``servo_offset_deg`` - the Ackermann model
  ``R = L / tan(k * angle + offset)``, calibrated on the real car
* ``actuator`` - whether the commanded speed comes from the firmware's target
  speed (default: the cruise control's own setpoint) or from the motor PWM
  through a duty->speed constant.
"""

from __future__ import annotations

import asyncio
import math
import pathlib
import re
import time
from dataclasses import dataclass, field
from typing import Optional

from .agsg_process import FirmwareProcess
from .hal import TelemetryState

SENSOR_PERIOD = 1.0 / 50.0  # the firmware polls sonar/camera at 50 Hz
CONTROL_STALE_S = 0.4

# The firmware's own telemetry frame, as pasted by telemetry.py:
#   x,y,heading,front,rear,left,right*CS   (metres, degrees, metres)
# It is already written for the visualiser, so it is the authority on the
# world<->firmware coordinate conversion. Do not re-derive it here.
TELEMETRY_RE = re.compile(
  r"^(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),"
  r"(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)\*([0-9A-Fa-f]{2})$"
)


# Lock-step physics step. Five milliseconds divides the firmware's 20 ms control
# period exactly, so it keeps its 50 Hz loop and the world still integrates finer
# than the 60 Hz display grid.
LOCKSTEP_STEP_MS = 5.0


@dataclass
class Config:
  actuator: str = "target_speed"  # "target_speed" | "pwm"
  pwm_cm_per_duty: float = 150.0  # only used by actuator="pwm"
  wheelbase_m: float = 0.070
  servo_gain: float = 1.0
  servo_offset_deg: float = 0.0
  servo_sign: float = 1.0
  imu_sign: float = -1.0
  imu_offset_deg: float = 0.0  # mounting offset: 0 = the firmware's zero is the boot heading
  min_radius_m: float = 0.05
  straight_deg: float = 0.7
  verbose: bool = False
  # Lock-step: the world advances only when the firmware blocks in a sleep, one
  # physics step at a time. A run is then a function of the world the firmware
  # saw, not of how fast the machine drew frames - which is what makes a sweep
  # mean anything.
  lockstep: bool = False
  extra: dict = field(default_factory=dict)

  @property
  def maps(self) -> dict:
    return {
      "actuator": self.actuator,
      "wheelbase_m": self.wheelbase_m,
      "servo_gain": self.servo_gain,
      "servo_offset_deg": self.servo_offset_deg,
      "servo_sign": self.servo_sign,
      "imu_sign": self.imu_sign,
    }


class AgsgController:
  """Runs the vendored firmware for as long as a UI is connected."""

  def __init__(self, server, config: Optional[Config] = None) -> None:
    self.server = server
    self.config = config or Config()
    self.proc: Optional[FirmwareProcess] = None
    self.last_act: Optional[dict] = None
    self.last_act_at = 0.0
    self.last_odom: Optional[dict] = None
    self.last_telemetry_pose: Optional[dict] = None
    self.last_sonar: Optional[dict] = None
    self.last_camera: Optional[dict] = None
    self._last_scale: Optional[float] = 1.0
    self._sim_clock: Optional[float] = None
    self._sim_accum: float = 0.0
    self._last_poll: float = 0.0
    self.telemetry_lines = 0
    self._hooked = False
    self.restart_requested = False
    self.pending_overrides = {}

  # ------------------------------------------------------------------- lifecycle

  async def run(self) -> None:
    while True:
      await self.server.wait_connected()
      finished = await self._run_connected()
      if finished:
        await self._hold_after_programme()
      await asyncio.sleep(0.2)

  async def _hold_after_programme(self) -> None:
    """Keep the world as the firmware left it until the operator asks for a run.

    The real robot stops when ``main`` returns. Respawning the interpreter here
    put a second car on the field while the round was still running: at 1x the
    parking routine returns about a second before the bay hold completes, the
    fresh process booted mid-field, drove another lap and the round ended on the
    180 s limit with 1.8.3 instead of the park it had already made. At 4x the
    same firmware won the race against the bay hold, which is why the parking
    score looked like a property of the speed multiplier.
    """
    if not (self.server.is_connected() and not self.restart_requested):
      return
    await self.server.send_log(
      "AGSG firmware: programa terminou; o carro fica parado até «Repetir pista»"
    )
    while self.server.is_connected() and not self.restart_requested:
      await asyncio.sleep(0.1)

  async def _run_connected(self) -> bool:
    cfg = self.config
    if self.pending_overrides:
      # Conventions can be switched per round from the UI (?imu=1&servo=-1) so the
      # hardware pairing can be tried without restarting the server.
      for key in ("imu_sign", "servo_sign", "servo_gain", "servo_offset_deg", "actuator"):
        if key in self.pending_overrides:
          setattr(cfg, key, self.pending_overrides[key])
      self.pending_overrides = {}
    self.restart_requested = False
    revision = ""
    try:
      rev_file = pathlib.Path(__file__).resolve().parent.parent / "firmware" / "agsg" / ".sync-revision"
      if rev_file.is_file():
        revision = " revision %s" % rev_file.read_text().split()[0][:12]
    except Exception:  # noqa: BLE001 - the revision is documentation, not control
      revision = ""
    await self.server.send_log(
      "AGSG firmware: a arrancar (actuator=%s, wheelbase=%.0f mm, imu=%+g, servo=%+g%s)"
      % (cfg.actuator, cfg.wheelbase_m * 1000, cfg.imu_sign, cfg.servo_sign, revision)
    )

    server = self.server
    if not self._hooked:
      # Per-message hook: encoder deltas must not be lost between sensor ticks.
      server.on_telemetry = self._on_telemetry
      self._hooked = True

    self.proc = FirmwareProcess(on_message=self._on_firmware, on_exit=self._on_exit)
    await self.proc.start()

    # A fresh process starts on its own foot: its clock is at 1x until the first
    # frame tells it otherwise, and the simulated-millisecond accumulator must
    # not carry a backlog from the previous process into this one.
    self._sim_clock = None
    self._sim_accum = 0.0
    self._last_scale = None
    # Lock-step bookkeeping: the firmware blocked in a sleep, and whether a step
    # is already in flight.
    self._fw_waiting = False
    self._step_pending = False
    self._steps_sent = 0
    if self.config.lockstep:
      await self.server.send_sync({"type": "lockstep", "on": True})
      # Prime the loop: the firmware's clock is only slaved to the simulated one
      # once it has seen a frame carrying it, and until then its sleeps do not
      # report "waiting" - so the very first step cannot wait for a signal that
      # can only exist after the first step.
      await self._tick()

    # Give the interpreter a moment to import the firmware and start its
    # background thread before the first frames arrive.
    for _ in range(100):
      if self.proc.ready or not self.proc.running:
        break
      await asyncio.sleep(0.02)

    if not self.proc.running:
      await server.send_log("AGSG firmware: falhou a arrancar (%s)" % (self.proc.exit_note or "sem detalhe"))
      return False

    # Wait for the world's sensor stream to actually flow before releasing the
    # state machine. The firmware's first decision is the round direction from
    # the side sonars: started before the browser streams them, it reads -1
    # everywhere and defaults to the mirrored (ccw) exit. Heading is the same
    # race - it reads 0 (its own default) until the first "heading" frame.
    state = server.state
    ready = False
    for _ in range(1500):  # up to ~30 s of wall time at 20 ms
      if state.heading_deg is not None and state.distances:
        ready = True
        break
      if self.restart_requested or not server.is_connected():
        break
      await asyncio.sleep(0.02)
    if not ready:
      await server.send_log(
        "AGSG firmware: stream de sensores nao chegou; a arrancar mesmo assim"
      )

    # Tell the clock where it is before releasing the state machine, so the
    # firmware's first ``sleep`` and its boot-time odometry already run in
    # simulated time. Sending "go" first and the clock afterwards is what made
    # the opening second of every 4x round last four simulated seconds.
    boot_scale = getattr(server.state, "sim_scale", 1.0) or 1.0
    self.proc.send({"t": "clock", "scale": boot_scale})
    self._last_scale = boot_scale
    self.proc.send({"t": "go"})
    await server.send_log("AGSG firmware: a correr")

    try:
      while (
        server.is_connected()
        and self.proc is not None
        and self.proc.running
        and not self.restart_requested
      ):
        # The 50 Hz sensor cadence is the firmware's own 20 ms control period, so
        # it is paced by the SIMULATED clock, not by asyncio.sleep. Sleeping
        # SENSOR_PERIOD/scale works on average but any scheduling hiccup becomes
        # 2x or 4x bigger in simulated time, and the firmware then steers on a
        # stale heading - which is how a corner that is clean at 1x gets clipped at
        # 4x. Counting simulated milliseconds gives exactly 50 updates per
        # simulated second whatever the multiplier and whatever the jitter.
        if self.config.lockstep:
          await self._lockstep_cycle()
          continue
        await asyncio.sleep(0.005)
        wall_now = asyncio.get_running_loop().time()
        wall_ms = (wall_now - self._last_poll) * 1000.0 if self._last_poll else 5.0
        self._last_poll = wall_now
        clock = getattr(self.server.state, "sim_clock_ms", None)
        sim_ms = 0.0
        if clock is not None:
          if self._sim_clock is None:
            self._sim_clock = clock
          sim_ms = max(0.0, clock - self._sim_clock)
          self._sim_clock = clock
        # While the simulator is not running its clock does not move - and it only
        # starts once the firmware commands it, which needs sensor frames. Fall
        # back to wall time so the boot sequence can happen at all.
        self._sim_accum += min(sim_ms if sim_ms > 0 else wall_ms, 200.0)
        period_ms = SENSOR_PERIOD * 1000.0
        budget = 8  # never storm: at 4x a 20 ms poll legitimately owes 4 ticks
        while self._sim_accum >= period_ms and budget > 0:
          self._sim_accum -= period_ms
          budget -= 1
          await self._tick()
    finally:
      # Ended on its own programme returning, not on a reset and not because the
      # browser went away: that is the only case where the world must be held.
      ended_by_programme = (
        not self.restart_requested
        and server.is_connected()
        and (self.proc is None or not self.proc.running)
      )
      proc = self.proc
      self.proc = None
      if proc is not None:
        note = proc.exit_note
        await proc.stop()
        if note:
          await server.send_log("AGSG firmware: %s" % note)
    return ended_by_programme

  async def _lockstep_cycle(self) -> None:
    """Advance the world one physics step, then feed the firmware.

    The world does not move on its own here: it takes one step each time the
    firmware blocks in a sleep. So the firmware samples the same instants, in the
    same order, in every run, and a round is reproducible - the wall clock only
    decides how fast this loop goes.

    One step in flight at a time: the browser answers with "stepped" once the
    physics has advanced, and only then is the next command allowed.
    """
    if self._step_pending or (not self._fw_waiting and self._steps_sent > 0):
      await asyncio.sleep(0.001)
      return
    if not self.server.is_connected():
      await asyncio.sleep(0.01)
      return
    self._step_pending = True
    self._steps_sent += 1
    await self.server.send_sync({"type": "advance", "stepMs": LOCKSTEP_STEP_MS})

  async def _tick(self) -> None:
    proc = self.proc
    if proc is None:
      return
    state: TelemetryState = self.server.state
    cfg = self.config

    heading = state.heading_deg if state.heading_deg is not None else 0.0
    scale = getattr(state, "sim_scale", 1.0) or 1.0
    # The world's clock rides on every frame: the firmware slaves its own clock to
    # it, so a sleep of 20 ms lands on the same simulated tick whatever the
    # multiplier is. Wall-derived firmware time made the control period stretch
    # with the machine's scheduling and the same seed drove differently.
    sim_ms = getattr(state, "sim_clock_ms", None)
    # The multiplier rides on every frame instead of a separate one-shot
    # message: the firmware side then has no state to resync when its process
    # restarts, which is how a 4x round used to end up driving a firmware whose
    # clock was still at 1x.
    proc.send(
      {
        "t": "h",
        "deg": cfg.imu_sign * heading + cfg.imu_offset_deg,
        "s": scale,
        "simMs": sim_ms,
      }
    )
    self._last_scale = scale

    distances_cm = {}
    for key, value in state.distances.items():
      distances_cm[key.lower()] = value * 100.0
    if distances_cm:
      proc.send({"t": "d", "cm": distances_cm, "s": scale, "simMs": sim_ms})

    signs = []
    for sign in getattr(state, "signs", []) or []:
      signs.append(
        {
          "state": int(sign.get("state", 0)),
          "bearingDeg": float(sign.get("angleDeg", 0.0)) * cfg.imu_sign,
          "distance": float(sign.get("distance", 0.0)),
        }
      )
    proc.send({"t": "c", "signs": signs, "s": scale, "simMs": sim_ms})

    await self._send_control()

  async def _send_control(self) -> None:
    act = self.last_act
    if act is None:
      return
    age = time.monotonic() - self.last_act_at
    velocity = self._velocity_from(act) if age <= CONTROL_STALE_S else 0.0
    radius = self._radius_from(act.get("a", 0.0))
    odom = self.last_odom
    pose = None
    if self.last_telemetry_pose is not None:
      # Prefer the pose the firmware itself streams to the visualiser: their
      # telemetry.py already applies the frame conversion, so nothing here has to
      # guess at signs or mirrors.
      pose = dict(self.last_telemetry_pose)
    elif odom is not None:
      # Fallback: the raw odometry fields, converted by the UI.
      pose = {
        "x": odom["x"] / 100.0,
        "y": odom["y"] / 100.0,
        "headingDeg": odom["h"],
        "cw": odom["cw"],
        "frame": "firmware",
      }
    debug = {}
    if self.last_sonar:
      debug["sonar"] = self.last_sonar
    if self.last_camera:
      debug["camera"] = self.last_camera
    debug = debug or None
    await self.server.send_control(
      velocity=velocity, steering_radius=radius, odom=pose, debug=debug
    )

  # ------------------------------------------------------------------ conversions

  def _velocity_from(self, act: dict) -> float:
    if self.config.actuator == "pwm":
      duty = float(act.get("duty", 0.0))
      direction = int(act.get("dir", 0))
      speed_cms = duty * self.config.pwm_cm_per_duty * direction
      return speed_cms / 100.0
    return float(act.get("v", 0.0))

  def _radius_from(self, angle_deg: float) -> Optional[float]:
    cfg = self.config
    delta = cfg.servo_gain * float(angle_deg) + cfg.servo_offset_deg
    if abs(delta) < cfg.straight_deg:
      return None
    tangent = math.tan(math.radians(delta))
    if abs(tangent) < 1e-9:
      return None
    radius = cfg.wheelbase_m / tangent
    # Keep the sign: a positive angle turns one way, a negative angle the other.
    # Taking abs() here and reapplying servo_sign collapses every command to the
    # same direction, which makes the car orbit instead of steering.
    radius = math.copysign(max(cfg.min_radius_m, abs(radius)), radius)
    return radius * cfg.servo_sign

  # --------------------------------------------------------------------- callbacks

  def _on_telemetry(self, payload: dict) -> None:
    """Called for every world message; encoder pulses go straight through."""
    kind = payload.get("type")
    if kind == "stepped":
      # The world has advanced by one physics step: release the gate and give the
      # firmware its sensor frame for that instant.
      self._step_pending = False
      task = asyncio.create_task(self._tick())
      self._pending_ticks = getattr(self, "_pending_ticks", [])
      self._pending_ticks.append(task)
      self._pending_ticks = [t for t in self._pending_ticks if not t.done()]
      return
    if kind == "race":
      # A "nova pista" / "repetir pista" asks for a fresh interpreter; the same
      # message can carry sign-convention overrides for the next round.
      if payload.get("action") == "reset":
        # The world is about to restart from the parking pose. Throw away every
        # cached reading first: the new firmware process reads the *first* frame
        # the controller sends it, and with the cache intact that frame still
        # described the end of the previous round - the car at (189,16) facing
        # north instead of (196,16) facing south. Its boot geometry was then
        # built on the wrong pose and the opening manoeuvre became a U-turn into
        # the lane; on top of that the driving direction came out inverted.
        self.server.state.reset()
        self.last_act = None
        self.last_odom = None
        self.last_telemetry_pose = None
        self.last_sonar = None
        self.last_camera = None
        for key, alias in (
          ("imuSign", "imu_sign"),
          ("servoSign", "servo_sign"),
          ("servoGain", "servo_gain"),
          ("servoOffsetDeg", "servo_offset_deg"),
          ("actuator", "actuator"),
        ):
          if payload.get(key) is not None:
            self.pending_overrides[alias] = payload[key]
        self.restart_requested = True
      return
    if kind != "encoder":
      return
    try:
      distance_m = float(payload.get("distance", 0.0))
    except (TypeError, ValueError):
      return
    if self.proc is not None and self.proc.running:
      self.proc.send({"t": "e", "cm": distance_m * 100.0})

  def _on_firmware(self, payload: dict) -> None:
    kind = payload.get("t")
    if kind == "wait":
      self._fw_waiting = bool(payload.get("w"))
      return
    if kind == "act":
      self.last_act = payload
      self.last_act_at = time.monotonic()
      self.last_sonar = payload.get("sd")
      self.last_camera = payload.get("cb")
    elif kind == "odom":
      self.last_odom = payload
    elif kind == "tel":
      self.telemetry_lines += 1
      match = TELEMETRY_RE.match(payload.get("m", "").strip())
      if match:
        # Their pose, in their units, already converted for the UI.
        self.last_telemetry_pose = {
          "x": float(match.group(1)),
          "y": float(match.group(2)),
          "headingDeg": float(match.group(3)),
          "frame": "telemetry",
        }
      elif self.config.verbose and self.telemetry_lines % 20 == 0:
        self.server.queue_log(payload.get("m", ""))
    elif kind == "err":
      self.server.queue_log("AGSG erro: %s" % payload.get("m"))
    elif kind == "stderr":
      text = str(payload.get("m", ""))
      if text.strip():
        self.server.queue_log("AGSG stderr: %s" % text)
        print("[firmware]", text, flush=True)

  def _on_exit(self, code: int) -> None:
    if code not in (0, -15):
      self.server.queue_log("AGSG firmware terminou (code %s)" % code)
