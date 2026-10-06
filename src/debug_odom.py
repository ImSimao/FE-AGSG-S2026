# debug_odom.py — diagnose odometry drift (gyro vs encoder).
#
# Prints the gyro heading and the encoder distance in a loop, so you can see
# which one is drifting:
#   1) GYRO TEST: keep the robot STILL and watch 'drift' grow.
#      -> if drift grows over time, the gyro (game rotation vector) is drifting.
#   2) ENCODER TEST: push the robot exactly 100 cm.
#      -> 'd_enc' should read ~100 cm. A constant % error means wrong scale
#         (PPR / RELACAO / DIAMETRO_RODA); jumps mean missed counts.
#
# How to run: rename this file to main.py, or in the REPL run: import debug_odom

import time
from machine import freq
from compass import Compass
from encoder import Encoder

freq(int(3 * 100_000_000))

print("=== Odom debug: gyro vs encoder ===")
print("1) GYRO TEST: keep the robot STILL. Watch 'drift' grow => gyro yaw drift.")
print("2) ENCODER TEST: push the robot 100 cm. 'd_enc' should read ~100 cm.")
print()

h0 = Compass.heading()
d0 = Encoder.distance_cm()
t0 = time.ticks_ms()

print("t(s)   heading    drift(deg)   d_enc(cm)")

while True:
    t = time.ticks_diff(time.ticks_ms(), t0) / 1000.0

    h = Compass.heading()
    drift = h - h0
    if drift > 180:
        drift -= 360
    elif drift < -180:
        drift += 360

    d = Encoder.distance_cm() - d0

    print("{:6.1f}  {:8.2f}  {:10.2f}  {:9.2f}".format(t, h, drift, d))

    time.sleep_ms(500)
