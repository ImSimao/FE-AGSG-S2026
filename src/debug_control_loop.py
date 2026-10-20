# debug_control_loop.py — trace the "backward odometry" bug.
#
# Runs move_coordinates (the FULL control loop) and logs the encoder distance,
# the motor PWM (positive = forward, negative = backward), the compass heading
# and the speeds. This shows whether the backward motion comes from the motor
# actually reversing (PWM negative) or from the compass/odometry (PWM positive
# but heading wrong).
#
# IMPORTANT: set state.clockwise to 1 or -1 to match the field direction.
#
# How to run: rename to main.py, or in the REPL: import debug_control_loop

import time
import _thread
import init  # starts the background loop (odometry + cruise control)
from state import state
from move_coordinates import move_coordinates
from encoder import Encoder
from background import cruiseControl

TARGET_CM = 100  # drive 100 cm forward in the relative frame

def main():
    state.clockwise = 1  # <-- set to 1 or -1 for your field direction

    print("=== Control loop trace (drive %.0f cm forward) ===" % TARGET_CM)
    print("ms, enc_cm, delta_cm, pwm, heading, target, speed")

    target_x = state.get_relative_odom[0] + TARGET_CM
    target_y = state.get_relative_odom[1]

    # run the move in the background, log on this thread
    _thread.start_new_thread(move_coordinates, (target_x, target_y))

    start = time.ticks_ms()
    last_enc = Encoder.distance_cm()
    while True:
        time.sleep_ms(50)
        now = time.ticks_ms()
        enc = Encoder.distance_cm()
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
    print("=== done ===")

if __name__ == "__main__":
    main()
