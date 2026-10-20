# debug_pid.py — steering PID step response (cross-track error).
#
# Drives the robot toward a target point that is offset from the current line,
# and logs the cross-track error at each control step so the overshoot, settling
# time and steady-state error can be computed for the journal §4.9.
#
# How to run: rename this file to main.py, or in the REPL: import debug_pid

import time
import math
import init  # starts the background loop (odometry + motor)
from state import state
from servo import Servo
from pid import PIDController
from telemetry import Telemetry

# Target: drive 100 cm forward with a 10 cm lateral step (induces a cross-track error).
TARGET_LEN = 100.0
TARGET_OFFSET = 10.0
MAX_SPEED = 30.0
LOOP_SLEEP_S = 0.02  # 20 ms -> 50 Hz

def main():
    print("=== PID step response ===")
    print("time_ms, cross_track_error_cm, servo_angle")

    x0, y0 = state.get_relative_odom
    A = (x0, y0)
    B = (x0 + TARGET_LEN, y0 + TARGET_OFFSET)

    ABx = B[0] - A[0]
    ABy = B[1] - A[1]
    L = math.sqrt(ABx * ABx + ABy * ABy)
    ux = ABx / L
    uy = ABy / L
    ux_perp = -uy
    uy_perp = ux

    pid = PIDController(kp=0.35, ki=0.003, kd=0.5)
    start = time.ticks_ms()
    last = start

    while True:
        now = time.ticks_ms()
        dt = time.ticks_diff(now, last) / 1000.0
        last = now

        C = state.get_relative_odom
        ACx = C[0] - A[0]
        ACy = C[1] - A[1]
        x_prime = ACx * ux + ACy * uy
        y_prime = ACx * ux_perp + ACy * uy_perp
        remaining = L - x_prime

        if remaining < 0:
            state.target_speed = 0
            Servo.set_angle(0)
            break

        state.target_speed = MAX_SPEED if remaining > 30 else (remaining / 30.0) * (MAX_SPEED - 5) + 5
        servo_angle = pid.compute(-y_prime, dt)
        Servo.set_angle(servo_angle * state.clockwise)

        elapsed = time.ticks_diff(now, start)
        print("%d, %.2f, %.2f" % (elapsed, -y_prime, servo_angle))

        time.sleep(LOOP_SLEEP_S)

    state.target_speed = 0
    Servo.set_angle(0)
    print("=== done ===")

if __name__ == "__main__":
    main()
