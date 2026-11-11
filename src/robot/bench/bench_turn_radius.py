# Bench test: servo angle -> turning radius, measured on the real car.
#
# Copy this file to the Pico 2 next to the firmware (it imports the firmware's own
# modules, nothing is modified) and run it with:
#
#     mpremote run bench_turn_radius.py
#
# The car drives the same arc in both directions at each angle and prints the
# radius per revolution. Fill the table at the bottom and hand it back: it gives
# `k` and `delta0` for  R = L / tan(k * angle + delta0)  with L = 70 mm.
#
# Safety: put the car on the floor with ~1 m of clear space, prop the driving
# wheels off the ground first to check the direction, and be ready to lift it.

import math
import time

import init  # noqa: F401  - starts the firmware's background control thread
from compass import Compass
from encoder import Encoder
from motor import Motor
from servo import Servo
from state import state

ANGLES = (29, 15)  # degrees of servo angle to test
SPEED = 20  # cm/s, the speed the firmware uses in tight manoeuvres
REVS = 2  # revolutions per measurement
TIMEOUT_MS = 30_000


def _unwrap(previous, current):
  delta = current - previous
  while delta > 180:
    delta -= 360
  while delta < -180:
    delta += 360
  return delta


def measure(angle):
  """Drive a full circle at a fixed servo angle; return the radius in mm."""
  Servo.set_angle(angle)
  time.sleep(0.5)
  start_cm = Encoder.distance_cm()
  previous = Compass.heading()
  swept = 0.0
  turns = 0
  radii = []
  state.target_speed = SPEED
  started = time.ticks_ms()

  while turns < REVS and time.ticks_diff(time.ticks_ms(), started) < TIMEOUT_MS:
    current = Compass.heading()
    swept += _unwrap(previous, current)
    previous = current
    if abs(swept) >= 360.0 * (turns + 1):
      turns += 1
      distance_cm = Encoder.distance_cm() - start_cm
      radius_mm = distance_cm / (abs(swept) * math.pi / 180.0) * 10.0
      radii.append(radius_mm)
      print("  rev %d: D=%6.1f cm  theta=%7.1f deg  R=%6.1f mm" % (turns, distance_cm, swept, radius_mm))
    time.sleep_ms(20)

  state.target_speed = 0
  Motor.brake()
  Servo.set_angle(0)
  time.sleep(1)
  if not radii:
    print("  no full revolution in %d ms - increase the space or the speed" % TIMEOUT_MS)
    return None
  return sum(radii) / len(radii)


def main():
  print("\n=== turn radius bench test (wheelbase 70 mm) ===")
  print("angle | direction | radius (mm) | implied delta_eff (deg)")
  results = {}
  for angle in ANGLES:
    for direction, signed in (("left", angle), ("right", -angle)):
      print("\nservo %+d deg, %s:" % (signed, direction))
      radius = measure(signed)
      if radius is None:
        continue
      results[(angle, direction)] = radius
      delta_eff = math.degrees(math.atan2(70.0, radius))
      print("  mean R = %6.1f mm  ->  delta_eff = %5.1f deg" % (radius, delta_eff))

  print("\n=== copy this back ===")
  for (angle, direction), radius in results.items():
    print("angle=%2d %-5s R=%6.1f mm" % (angle, direction, radius))

  # Two points, two unknowns: delta_eff = k * angle + delta0
  if (ANGLES[0], "left") in results and (ANGLES[1], "left") in results:
    r1 = results[(ANGLES[0], "left")]
    r2 = results[(ANGLES[1], "left")]
    e1 = math.degrees(math.atan2(70.0, r1))
    e2 = math.degrees(math.atan2(70.0, r2))
    k = (e1 - e2) / (ANGLES[0] - ANGLES[1])
    delta0 = e1 - k * ANGLES[0]
    print("\nfit: delta_eff = %.3f * angle + %.2f  ->  --servo-gain %.3f --servo-offset-deg %.2f" % (k, delta0, k, delta0))


if __name__ == "__main__":
  main()
