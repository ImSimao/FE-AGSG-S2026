# debug_control_loop.py — trace the control loop.
#
# Drives forward (constant speed + deceleration) on the main core while the
# background loop (core 1) runs the odometry + cruise control. Logs the encoder
# distance, motor PWM, compass heading and speeds.
#
# IMPORTANT: set state.clockwise to 1 or -1 to match the field direction.
#
# How to run: rename to main.py, or in the REPL: import debug_control_loop

import time
import init  # starts the background loop (odometry + cruise control)
from state import state
from encoder import Encoder
from background import cruiseControl

TARGET_CM = 200       # total distance to drive
MAX_SPEED = 80        # cm/s (match move_coordinates; try 60 if needed)
MIN_SPEED = 12        # cm/s at the end of the deceleration (motor cannot creep below ~12)
DECEL_CM = 50         # start decelerating this far from the target
BRAKE_DECEL = 120.0   # cm/s^2, braking deceleration used for the brake anticipation

def main():
    state.clockwise = 1  # <-- set to 1 or -1 for your field direction

    print("=== Control loop trace (drive %.0f cm) ===" % TARGET_CM)
    print("ms, enc_cm, delta_cm, pwm, heading, target, speed")

    start = time.ticks_ms()
    start_enc = Encoder.distance_cm()
    last_enc = start_enc
    last_print = start

    while True:
        time.sleep_ms(10)
        now = time.ticks_ms()

        enc = Encoder.distance_cm()
        travelled = enc - start_enc
        remaining = TARGET_CM - travelled

        if remaining <= 0:
            break

        if remaining < DECEL_CM:
            state.target_speed = remaining / DECEL_CM * (MAX_SPEED - MIN_SPEED) + MIN_SPEED
        else:
            state.target_speed = MAX_SPEED

        if time.ticks_diff(now, last_print) >= 50:
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
            last_print = now

        if time.ticks_diff(now, start) > 15000:
            break

    state.target_speed = 0
    print("=== done ===")

if __name__ == "__main__":
    main()
