from distance import Distance
import time
import math
from compass import Compass
from encoder import Encoder
from state import state


SENSOR_INTERVAL_MS = 1/50 * 1000  # 50 Hz
ODOM_INTERVAL_MS = 1/100 * 1000    # 100 Hz
LOOP_SLEEP_MS = 1


def _read_distance_sensors():
    Distance.get_sensor_data()


def _update_odometry(last_distance_cm):
    compass_angle = Compass.heading()
    state.compass_angle = compass_angle

    current_distance_cm = Encoder.distance_cm()
    delta_cm = current_distance_cm - last_distance_cm
    theta_rad = math.radians(compass_angle)

    state.odom_x += delta_cm * math.cos(theta_rad)
    state.odom_y += delta_cm * math.sin(theta_rad)
    return current_distance_cm


def background_task():
    last_distance_cm = Encoder.distance_cm()
    last_sensor_ms = time.ticks_ms()
    last_odom_ms = last_sensor_ms

    print("Background task started")

    while True:
        now_ms = time.ticks_ms()

        if time.ticks_diff(now_ms, last_sensor_ms) >= SENSOR_INTERVAL_MS:
            _read_distance_sensors()
            last_sensor_ms = now_ms

        if time.ticks_diff(now_ms, last_odom_ms) >= ODOM_INTERVAL_MS:
            last_distance_cm = _update_odometry(last_distance_cm)
            last_odom_ms = now_ms

        time.sleep_ms(LOOP_SLEEP_MS)