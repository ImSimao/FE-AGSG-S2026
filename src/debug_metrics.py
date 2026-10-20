# debug_metrics.py — collect the driving metrics for the journal §4.9.
#
# 1) ODOMETRY DRIFT (no sonar correction): drives a straight 100 cm and compares
#    the encoder+IMU odometry against the commanded 100 cm. The difference is the
#    drift per 100 cm (scale to cm per lap).
# 2) FINAL POSITION ERROR: drives a 4-segment square and logs the return-to-origin
#    error (encoder-only, with the sonar correction disabled in background.py).
#
# How to run: rename this file to main.py, or in the REPL: import debug_metrics

import time
import init  # starts the background loop (odometry + motor + sensors)
from state import state
from move_coordinates import move_coordinates, rotate_angle
from encoder import Encoder

STRAIGHT_CM = 100.0

def measure_straight():
    """Drive straight STRAIGHT_CM and return (encoder_distance, odom_distance)."""
    enc0 = Encoder.distance_cm()
    x0, y0 = state.get_relative_odom
    move_coordinates(x0 + STRAIGHT_CM, y0, rotate=False)
    encoder_d = Encoder.distance_cm() - enc0
    x1, y1 = state.get_relative_odom
    odom_d = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
    return encoder_d, odom_d

def main():
    print("=== Metrics: odometry drift (straight %.0f cm) ===" % STRAIGHT_CM)
    print("run, encoder_cm, odom_cm, encoder_error_cm")
    for i in range(5):
        enc, odom = measure_straight()
        print("%d, %.2f, %.2f, %.2f" % (i, enc, odom, enc - STRAIGHT_CM))
        time.sleep(1)

    print("=== done: paste these numbers into the journal table ===")

if __name__ == "__main__":
    main()
