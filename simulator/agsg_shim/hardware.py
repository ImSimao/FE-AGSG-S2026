"""Simulated hardware state for the AGSG firmware shim.

One object holds every signal:

* the bridge writes sensor readings **into** it,
* the shim modules (``machine.py``, ``bno08x.py``) read from it,
* the runner samples the actuators **out** of it.

Nothing here knows about the browser, websockets or the firmware's own logic.
Units are the ones the firmware expects (cm for distance, degrees for heading),
so any conversion lives at the bridge, where it is visible.
"""

from __future__ import annotations

import math
import threading

# SRF04-class sensors saturate around 250 cm; the firmware maps 0xFFFF to -1.0.
SRF04_MAX_CM = 250.0
INVALID = -1.0
SONAR_HEADER = 0xA5
SONAR_ORDER = ("front", "rear", "left", "right")

# OpenMV H7 runs QQVGA (160x120) with vflip+hmirror, so the image is a 180 deg
# rotation of the physical scene.
CAM_WIDTH = 160
CAM_HEIGHT = 120
# A traffic sign is 5 cm across and 5 cm tall, so its blob is
# CAM_FOCAL_PX * 5 / distance cm wide and high, and its area is that squared.
#
# The detector model is not invented: `firmware/agsg/Camera/main.py` (the team's
# own OpenMV script) runs
#
#     find_blobs(THRESHOLDS, roi=ROI, merge=False,
#                pixels_threshold=80, area_threshold=80)
#
# so a sign is only reported at all when its blob covers at least 80 pixels -
# about 8.9 px across, i.e. **closer than ~63 cm** for a 5 cm face. Modelling
# that here is the difference between a simulator that feeds the firmware signs
# the robot would never see (a 2.5 m sign matched a position 45 cm ahead, and a
# 114 cm sign was written into a zone 57 cm away) and one that feeds it what the
# camera actually reports.
SIGN_WIDTH_CM = 5.0
MIN_BLOB_AREA_PX = 80.0
# Focal length in pixels implied by the sensor's horizontal field of view:
# the OpenMV Cam H7 with a 2.8 mm lens sees 70.8 deg across, so a blob at
# bearing b lands at x = W/2 + f*tan(b) with f = (W/2)/tan(35.4 deg). Keeping
# the projection derived from the FOV instead of a magic 114 px means the frame
# and the visible-field test in the UI cannot drift apart.
HALF_FOV_DEG = 70.8 / 2.0
CAM_FOCAL_PX = (CAM_WIDTH / 2.0) / math.tan(math.radians(HALF_FOV_DEG))


def apparent_width_px(distance_m: float) -> float:
  """How wide the sign's blob is in the frame at that distance."""
  distance_cm = max(1.0, float(distance_m or 0.0) * 100.0)
  return CAM_FOCAL_PX * SIGN_WIDTH_CM / distance_cm


class Hal:
  """Every simulated signal in one place."""

  def __init__(self) -> None:
    self.lock = threading.RLock()
    self.cpu_hz = 300_000_000

    # --- sensors (world -> firmware)
    self.heading_deg = 0.0
    self.enc_pending_cm = 0.0
    self.distances = {key: INVALID for key in SONAR_ORDER}
    self.signs = []  # [{"state": 1|2, "bearing_deg": float, "distance_m": float}]
    self.camera_frame = b"CAM:\n"

    # --- buses
    self.rx = {}  # uart id -> bytearray of bytes waiting to be read
    self.tx = []  # lines the firmware wrote (telemetry / log)

    # --- actuators (firmware -> world)
    self.pwm = {}  # pin id -> PwmShim
    self.levels = {}  # pin id -> 0/1 for output pins
    self.irq_handlers = {}

    # --- lifecycle
    self.started = False
    self.stopping = False

  # ------------------------------------------------------------------ sensors

  def set_heading(self, deg: float) -> None:
    with self.lock:
      self.heading_deg = float(deg) % 360.0

  def add_travel_cm(self, cm: float) -> None:
    """Accumulate forward travel (negative = reverse) for the encoder injector."""
    with self.lock:
      self.enc_pending_cm += float(cm)

  def take_travel_cm(self) -> float:
    with self.lock:
      value = self.enc_pending_cm
      self.enc_pending_cm = 0.0
      return value

  def set_distances_cm(self, distances: dict) -> None:
    """Store the four ranges and rebuild the framed sonar packet."""
    with self.lock:
      for key in SONAR_ORDER:
        raw = distances.get(key, INVALID)
        try:
          value = float(raw)
        except (TypeError, ValueError):
          value = INVALID
        self.distances[key] = value
      packet = self._sonar_packet()
      self.rx[0] = bytearray(packet)

  def set_signs(self, signs) -> None:
    """Store the visible traffic signs and rebuild one camera frame."""
    with self.lock:
      clean = []
      for sign in signs or []:
        try:
          state = int(sign.get("state", 0))
        except (TypeError, ValueError):
          state = 0
        if state not in (1, 2):
          continue
        try:
          bearing = float(sign.get("bearingDeg", sign.get("bearing_deg", 0.0)))
          distance = float(sign.get("distance", 0.0))
        except (TypeError, ValueError):
          continue
        width_px = apparent_width_px(distance)
        if width_px * width_px < MIN_BLOB_AREA_PX:   # square face: area = w * h
          continue
        clean.append({"state": state, "bearing_deg": bearing, "distance_m": distance,
                      "width_px": width_px})
      self.signs = clean
      self.camera_frame = self._camera_frame()
      # The camera UART is a one-way bus: the frame has to sit in the receive
      # buffer, exactly like the sonar packet sits in UART0's.
      self.rx[1] = bytearray(self.camera_frame)

  # ------------------------------------------------------------------- framing

  def _sonar_packet(self) -> bytes:
    body = bytearray()
    for key in SONAR_ORDER:
      cm = self.distances[key]
      if cm is None or cm < 0 or cm > SRF04_MAX_CM:
        raw = 0xFFFF
      else:
        raw = int(round(cm * 10.0))
        raw = max(0, min(0xFFFE, raw))
      body += int(raw).to_bytes(2, "little")
    checksum = 0
    for byte in body:
      checksum ^= byte
    return bytes([SONAR_HEADER]) + bytes(body) + bytes([checksum & 0xFF])

  def _camera_frame(self) -> bytes:
    """Project the visible signs into the OpenMV wire format.

    The firmware gate is ``hypot(cx - 80, cy - 120) <= 65`` measured on the
    *bottom centre* of the blob, so the projection has to stay in that window
    to reproduce what the camera really reports.
    """
    parts = []
    for sign in self.signs:
      color = "GREEN" if sign["state"] == 1 else "RED"
      bearing = math.radians(sign["bearing_deg"])
      # physical bearing -> image column, then hmirror/vflip (180 deg rotation)
      x_phys = CAM_WIDTH / 2.0 + CAM_FOCAL_PX * math.tan(bearing)
      x_img = (CAM_WIDTH - 1) - x_phys
      cx = int(max(0, min(CAM_WIDTH - 1, round(x_img))))
      cy = CAM_HEIGHT - 6  # blob bottom near the bottom edge of the frame
      parts.append("{},{},{}".format(cx, cy, color))
    line = "CAM:" + ";".join(parts) + "\n"
    return line.encode("ascii")

  # ---------------------------------------------------------------- actuators

  def motor_state(self) -> dict:
    """Direction word from the TB6612 control pins (1/2) and the PWM (0)."""
    with self.lock:
      in1 = self.levels.get(1, 0)
      in2 = self.levels.get(2, 0)
      pwm = self.pwm.get(0)
      duty = float(pwm.duty) if pwm is not None else 0.0
      brake = bool(in1 and in2)
      direction = 1 if (in2 and not in1) else (-1 if (in1 and not in2) else 0)
      return {"duty": duty, "direction": direction, "brake": brake}

  def servo_duty(self) -> float:
    with self.lock:
      pwm = self.pwm.get(3)
      return float(pwm.duty) if pwm is not None else 0.0


hal = Hal()
