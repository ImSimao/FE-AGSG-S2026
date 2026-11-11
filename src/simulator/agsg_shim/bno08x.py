"""BNO085 driver stub.

The firmware only uses the game rotation vector and reads ``euler_heading``.
Emulating the I2C register map would test this stub rather than the firmware,
so the driver is replaced wholesale and the heading comes from the world.
"""

from __future__ import annotations

from hardware import hal

BNO_REPORT_ACCELEROMETER = 0x01
BNO_REPORT_GYROSCOPE = 0x02
BNO_REPORT_MAGNETOMETER = 0x03
BNO_REPORT_LINEAR_ACCELERATION = 0x04
BNO_REPORT_ROTATION_VECTOR = 0x05
BNO_REPORT_GRAVITY = 0x06
BNO_REPORT_GAME_ROTATION_VECTOR = 0x08
BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR = 0x09
BNO_REPORT_STEP_COUNTER = 0x11
BNO_REPORT_STABILITY_CLASSIFIER = 0x13
BNO_REPORT_RAW_ACCELEROMETER = 0x14
BNO_REPORT_RAW_GYROSCOPE = 0x15
BNO_REPORT_RAW_MAGNETOMETER = 0x16

__all__ = [
  "BNO08X",
  "BNO_REPORT_GAME_ROTATION_VECTOR",
  "BNO_REPORT_ROTATION_VECTOR",
  "BNO_REPORT_ACCELEROMETER",
  "BNO_REPORT_GYROSCOPE",
  "BNO_REPORT_MAGNETOMETER",
  "BNO_REPORT_LINEAR_ACCELERATION",
  "BNO_REPORT_GRAVITY",
  "BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR",
  "BNO_REPORT_STEP_COUNTER",
  "BNO_REPORT_STABILITY_CLASSIFIER",
  "BNO_REPORT_RAW_ACCELEROMETER",
  "BNO_REPORT_RAW_GYROSCOPE",
  "BNO_REPORT_RAW_MAGNETOMETER",
]


class BNO08X:
  def __init__(self, i2c, reset_pin=None, debug=False, **kwargs):
    self.i2c = i2c
    self.debug = debug

  # -- configuration calls the firmware makes at import time
  def set_quaternion_euler_vector(self, report):
    pass

  def enable_feature(self, feature, freq=None, **kwargs):
    pass

  def disable_feature(self, feature):
    pass

  # -- readings
  @property
  def euler_heading(self):
    return hal.heading_deg

  @property
  def euler_roll(self):
    return 0.0

  @property
  def euler_pitch(self):
    return 0.0

  def get_quaternion(self):
    return (0.0, 0.0, 0.0, 1.0)
