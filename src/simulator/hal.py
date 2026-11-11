from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Dict, Optional


@dataclass
class Pose:
  x: float
  y: float
  headingDeg: float


@dataclass
class SignDetection:
  lane: str
  state: int
  distance: float
  angleDeg: float


class TelemetryState:
  """Mutable snapshot storage shared across the HAL."""

  def __init__(self) -> None:
    self.heading_deg: Optional[float] = None  # relative
    self.heading_abs_deg: Optional[float] = None
    self.encoder_total_m: float = 0.0
    self.distances: Dict[str, float] = {}
    self.sign: Optional[SignDetection] = None
    self.signs: list = []
    self.timestamp_ms: Optional[int] = None
    self.sim_scale: float = 1.0
    self.sim_clock_ms: Optional[float] = None
    self._last_enc_ms: Optional[int] = None
    self._last_enc_speed: Optional[float] = None
    self.odom: Pose = Pose(0.0, 0.0, 0.0)

  def reset(self) -> None:
    """Clear telemetry to a safe default when a client disconnects."""
    self.heading_deg = None
    self.heading_abs_deg = None
    self.encoder_total_m = 0.0
    self._last_enc_ms = None
    self._last_enc_speed = None
    self.distances.clear()
    self.sign = None
    self.signs = []
    self.timestamp_ms = None
    self.odom.x = 0.0
    self.odom.y = 0.0
    self.odom.headingDeg = 0.0

  def update_from_message(self, data: Dict) -> None:
    """Update sensor state from a WS telemetry message."""
    msg_type = data.get("type")
    ts = data.get("timestampMs") or data.get("ts")
    if _is_number(ts):
      self.timestamp_ms = int(ts)
    scale = data.get("scale")
    if _is_number(scale) and float(scale) > 0:
      self.sim_scale = float(scale)
    clock = data.get("simClockMs")
    if _is_number(clock):
      self.sim_clock_ms = float(clock)
    if msg_type == "heading":
      heading_rel = data.get("headingDeg")
      heading_abs = data.get("headingDegAbs")
      if _is_number(heading_rel):
        self.heading_deg = float(heading_rel)
      if _is_number(heading_abs):
        self.heading_abs_deg = float(heading_abs)
      if self.heading_deg is not None:
        self.odom.headingDeg = self.heading_deg
      elif self.heading_abs_deg is not None:
        self.odom.headingDeg = self.heading_abs_deg
    elif msg_type == "encoder":
      distance = data.get("distance")
      if _is_number(distance):
        self._apply_encoder(float(distance))
    elif msg_type == "ultrasonic":
      distances = data.get("distances") or {}
      self.distances = {k: float(v) for k, v in distances.items() if _is_number(v)}
    elif msg_type == "sign":
      sign = data.get("sign")
      if sign and {"lane", "state", "distance", "angleDeg"} <= sign.keys():
        self.sign = SignDetection(
          lane=str(sign["lane"]),
          state=int(sign["state"]),
          distance=float(sign["distance"]),
          angleDeg=float(sign["angleDeg"]),
        )
        self.signs = [sign]
      else:
        self.sign = None
        self.signs = []
    elif msg_type == "signs":
      # Every sign inside the camera's field of view, not just the nearest one:
      # the firmware's own selection logic must decide, not the simulator.
      raw = data.get("signs") or []
      cleaned = [
        s
        for s in raw
        if isinstance(s, dict) and {"lane", "state", "distance", "angleDeg"} <= s.keys()
      ]
      cleaned.sort(key=lambda s: float(s["distance"]))
      self.signs = cleaned
      if cleaned:
        first = cleaned[0]
        self.sign = SignDetection(
          lane=str(first["lane"]),
          state=int(first["state"]),
          distance=float(first["distance"]),
          angleDeg=float(first["angleDeg"]),
        )
      else:
        self.sign = None

  # -- wheel slip ---------------------------------------------------------
  # A DC motor on a 3:5 gearbox does not report the distance the car travelled,
  # it reports how far the wheel turned. Under acceleration the tyre spins on
  # the floor and the encoder over-reads; under hard braking it drags and
  # under-reads. The over-read is proportional to the speed change, so the model
  # is one extra length per (m/s) of dv, capped as a fraction of the step.
  # Doubled from 0.015/0.15/0.0002: a 3:5 gearbox on a light chassis, with the
  # wheels turned, spins well before it grips. The old numbers made the odometry
  # better than the hardware can be, which hid errors the firmware has to survive.
  ENCODER_SLIP_GAIN_S = 0.030
  ENCODER_SLIP_MAX_FRACTION = 0.25
  ENCODER_NOISE_M = 0.0006  # quantisation + vibration, a few counts on a 200 ppr wheel

  def _slip_m(self, distance: float, dt_s: float | None) -> float:
    if dt_s is None or dt_s <= 0.0:
      self._last_enc_speed = None
      return 0.0
    speed = distance / dt_s
    previous = self._last_enc_speed
    self._last_enc_speed = speed
    if previous is None:
      return 0.0
    delta_v = speed - previous
    extra = self.ENCODER_SLIP_GAIN_S * delta_v
    cap = self.ENCODER_SLIP_MAX_FRACTION * abs(distance)
    extra = max(-cap, min(cap, extra))
    return extra + random.gauss(0.0, self.ENCODER_NOISE_M)

  def _apply_encoder(self, distance: float) -> None:
    dt_s = None
    if self._last_enc_ms is not None and self.timestamp_ms > self._last_enc_ms:
      dt_s = (self.timestamp_ms - self._last_enc_ms) / 1000.0
    self._last_enc_ms = self.timestamp_ms
    distance = distance + self._slip_m(distance, dt_s)
    heading = self.heading_abs_deg
    if heading is None:
      heading = self.heading_deg
    if heading is None:
      heading = self.odom.headingDeg
    rad = math.radians(heading)
    self.odom.x += distance * math.cos(rad)
    self.odom.y += distance * math.sin(rad)
    self.encoder_total_m += distance


class HeadingSensor:
  def __init__(self, state: TelemetryState) -> None:
    self._state = state

  def get_heading_deg(self) -> Optional[float]:
    return self._state.heading_deg if self._state.heading_deg is not None else self._state.heading_abs_deg


class DistanceSensorArray:
  def __init__(self, state: TelemetryState) -> None:
    self._state = state

  def get_distances(self) -> Dict[str, float]:
    return dict(self._state.distances)


class TrafficSignSensor:
  def __init__(self, state: TelemetryState) -> None:
    self._state = state

  def get_detection(self) -> Optional[SignDetection]:
    return self._state.sign


class Hal:
  """Minimal HAL surface that can map to MicroPython later."""

  def __init__(self, state: TelemetryState) -> None:
    self.heading = HeadingSensor(state)
    self.distances = DistanceSensorArray(state)
    self.signs = TrafficSignSensor(state)
    self.odom = state.odom


def _is_number(value) -> bool:
  try:
    float(value)
    return True
  except (TypeError, ValueError):
    return False


