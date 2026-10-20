# debug_control_loop.py — trace the "backward odometry" bug.
#
# Drives forward (constant speed + deceleration) on the main core while the
# background loop (core 1) runs the odometry + cruise control. Logs the encoder
# distance, motor PWM (positive = forward, negative = backward), compass heading
# and speeds, so we can see whether the backward motion comes from the motor
# actually reversing (PWM negative) or from the compass/odometry.
#
# IMPORTANT: set state.clockwise to 1 or -1 to match the field direction.
#
# How to run: rename to main.py, or in the REPL: import debug_control_loop

import time
import init  # starts the background loop (odometry + cruise control)
from state import state
from encoder import Encoder
from background import cruiseControl

TARGET_CM = 100       # total distance to drive
MAX_SPEED = 30        # cm/s
MIN_SPEED = 5         # cm/s at the end of the deceleration
DECEL_CM = 30         # start decelerating this far from the target

def main():
    state.clockwise = 1  # <-- set to 1 or -1 for your field direction

    print("=== Control loop trace (drive %.0f cm) ===" % TARGET_CM)
    print("ms, enc_cm, delta_cm, pwm, heading, target, speed")

    start = time.ticks_ms()
    start_enc = Encoder.distance_cm()
    last_enc = start_enc

    while True:
        time.sleep_ms(50)
        now = time.ticks_ms()

        enc = Encoder.distance_cm()
        travelled = enc - start_enc
        remaining = TARGET_CM - travelled

        if remaining <= 0:
            break

        # trapezoidal speed profile (like move_coordinates)
        if remaining < DECEL_CM:
            state.target_speed = remaining / DECEL_CM * (MAX_SPEED - MIN_SPEED) + MIN_SPEED
        else:
            state.target_speed = MAX_SPEED

        print("%d, %.3f, %.3f, %.3f, %.1f, %.1f, %.1f" % (
            time.ticks_diff(now, start),
            enc,
            enc - last_enc,
            cruiseControl.current_pwm,
            state.compass_angle,
            state.target_speed,
            state.current_speed,
        ))
        last_enc = enc

        if time.ticks_diff(now, start) > 15000:
            break

    state.target_speed = 0
    time.sleep_ms(500)
    print("=== done ===")

if __name__ == "__main__":
    main()
