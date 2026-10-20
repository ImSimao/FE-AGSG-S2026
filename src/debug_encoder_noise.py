# debug_encoder_noise.py — isolate the encoder problem from the control loop.
#
# Runs the motor FORWARD at a FIXED PWM (no cruise control, no odometry) and logs
# the encoder distance. This tells us where the fault is:
#
#   * encoder delta POSITIVE and smooth  -> encoder is fine; the bug is in the
#                                            cruise control (motor reversing).
#   * encoder delta NEGATIVE / erratic   -> the encoder itself is being corrupted
#                                            by PWM/motor noise.
#
# Run with the wheels in the air (or on the ground with room to drive).
# How to run: rename to main.py, or in the REPL: import debug_encoder_noise

import time
from machine import freq
from motor import Motor
from encoder import Encoder

freq(int(3 * 100_000_000))

PWM_DUTY = 0.30   # 30% forward
DURATION_S = 5

def main():
    print("=== Encoder noise test: motor forward at %.0f%% PWM ===" % (PWM_DUTY * 100))
    Motor.forward()
    Motor.ena.duty_u16(int(PWM_DUTY * 65535))

    start = time.ticks_ms()
    last = Encoder.distance_cm()
    print("ms, encoder_cm, delta_cm")

    while time.ticks_diff(time.ticks_ms(), start) < DURATION_S * 1000:
        now = time.ticks_ms()
        cur = Encoder.distance_cm()
        print("%d, %.3f, %.3f" % (now - start, cur, cur - last))
        last = cur
        time.sleep_ms(50)

    Motor.stop()
    Motor.ena.duty_u16(0)
    print("=== done ===")

if __name__ == "__main__":
    main()
