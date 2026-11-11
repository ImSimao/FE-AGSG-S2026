# debug_brake_distance.py — measure the robot's braking distance.
#
# Accelerates to TARGET_SPEED, cruises to TRIGGER_CM, then commands the stop
# (target_speed = 0 -> brake). It measures how far the robot travels AFTER the
# stop command (the braking distance) and derives the braking deceleration.
#
# Run once per speed (change TARGET_SPEED): 80, 60, 45, 30, 20 cm/s.
# Then the brake anticipation can use the measured curve.
#
# How to run: rename to main.py, or in the REPL: import debug_brake_distance

import time
import init  # starts the background loop (odometry + cruise control)
from state import state
from encoder import Encoder
from background import cruiseControl

TARGET_SPEED = 80     # cruise speed to test (cm/s)
TRIGGER_CM = 150      # command the stop once the robot has travelled this far
STOP_SPEED = 1.0      # consider stopped below this speed (cm/s)

def main():
    state.clockwise = 1  # <-- set to 1 or -1 for your field direction

    print("=== Braking distance test: cruise %.0f cm/s ===" % TARGET_SPEED)
    print("ms, enc_cm, speed, pwm")

    start = time.ticks_ms()
    start_enc = Encoder.distance_cm()
    last_print = start

    triggered = False
    trigger_enc = 0.0
    trigger_speed = 0.0

    while True:
        time.sleep_ms(10)
        now = time.ticks_ms()

        enc = Encoder.distance_cm()
        travelled = enc - start_enc

        if not triggered:
            state.target_speed = TARGET_SPEED
            if travelled >= TRIGGER_CM:
                triggered = True
                trigger_enc = enc
                trigger_speed = state.raw_speed
                state.target_speed = 0
                print("STOP COMMAND: enc=%.3f cm, raw speed=%.1f cm/s" % (enc, trigger_speed))
        else:
            state.target_speed = 0
            if abs(state.raw_speed) < STOP_SPEED:
                final_enc = Encoder.distance_cm()
                brake_dist = final_enc - trigger_enc
                print("FINAL enc=%.3f cm" % final_enc)
                print("BRAKING DISTANCE = %.3f cm" % brake_dist)
                if brake_dist > 0.01 and trigger_speed > 1.0:
                    decel = trigger_speed * trigger_speed / (2.0 * brake_dist)
                    print("BRAKE DECEL ~ %.0f cm/s^2  (v=%.1f, s=%.3f)" % (
                        decel, trigger_speed, brake_dist))
                break

        if time.ticks_diff(now, last_print) >= 50:
            print("%d, %.3f, %.1f, %.3f" % (
                time.ticks_diff(now, start),
                enc,
                state.raw_speed,
                cruiseControl.current_pwm,
            ))
            last_print = now

        if time.ticks_diff(now, start) > 20000:
            print("TIMEOUT")
            break

    state.target_speed = 0
    print("=== done ===")

if __name__ == "__main__":
    main()
