from distance import Distance
import time
import math
from compass import Compass
from encoder import Encoder
from motor import Motor
from pid import PIDController
from state import state


SENSOR_INTERVAL_MS = 1/50 * 1000  # 50 Hz
ODOM_INTERVAL_MS = 1/100 * 1000    # 100 Hz
SPEED_INTERVAL_MS = 1/10 * 1000    # 20 Hz
LOOP_SLEEP_MS = 1


class SpeedController:
    def __init__(self):
        # Output is normalized to PWM duty (0.0 to 1.0).
        self.pid = PIDController(
            kp=0.01,
            ki=0.005,
            kd=0.025,
            output_min=0.0,
            output_max=1.0,
            integral_limit=20.0,
        )

    def update(self, target_speed, current_speed, dt_s):
        if target_speed <= 0:
            self.pid.reset()
            return 0.0

        error = target_speed - current_speed
        duty = self.pid.compute(error, dt_s)
        duty = duty + (0.0127 * current_speed)
        if duty < 0.0:
            duty = 0.0
        elif duty > 1.0:
            duty = 1.0
        return duty


speed_controller = SpeedController()

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


def update_speed(last_distance_cm, last_speed_ms):
    current_distance_cm = Encoder.distance_cm()
    delta_cm = current_distance_cm - last_distance_cm
    delta_time_ms = time.ticks_diff(time.ticks_ms(), last_speed_ms)
    delta_time_s = delta_time_ms / 1000.0

    if delta_time_ms <= 0:
        return current_distance_cm

    state.current_speed = delta_cm / delta_time_s



    target_speed = state.target_speed
    current_speed = state.current_speed

    duty = speed_controller.update(target_speed, current_speed, delta_time_s)
    Motor.ena.duty_u16(int(duty * 65535))

    return current_distance_cm

def background_task():
    last_distance_cm = Encoder.distance_cm()
    last_speed_cm = last_distance_cm
    last_sensor_ms = time.ticks_ms()
    last_odom_ms = last_sensor_ms
    last_speed_ms = last_sensor_ms

    print("Background task started")

    while True:
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_sensor_ms) >= SENSOR_INTERVAL_MS:
            _read_distance_sensors()
            last_sensor_ms = now_ms


        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_odom_ms) >= ODOM_INTERVAL_MS:
            last_distance_cm = _update_odometry(last_distance_cm)
            last_odom_ms = now_ms


        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_speed_ms) >= SPEED_INTERVAL_MS:
            last_speed_cm = update_speed(last_speed_cm, last_speed_ms)
            last_speed_ms = now_ms

        time.sleep_ms(LOOP_SLEEP_MS)
        