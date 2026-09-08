from distance import Distance
from camera import Camera
import time
import math
from compass import Compass
from encoder import Encoder
from motor import Motor
from state import state
from telemetry import Telemetry


SENSOR_INTERVAL_MS    = 1/5  * 1000  # 50 Hz
CAMERA_INTERVAL_MS    = 1/200 * 1000  # 200 Hz
ODOM_INTERVAL_MS      = 1/100 * 1000  # 100 Hz
SPEED_INTERVAL_MS     = 1/10  * 1000  # 10 Hz
TELEMETRY_INTERVAL_MS = 1/20  * 1000  # 10 Hz
ADJUST_ODOM_INTERVAL_MS = 1/5  * 1000  # 1 Hz
LOOP_SLEEP_MS = 1



class CruiseControl:
    def __init__(self, max_pwm=1, min_pwm=-1,
                 max_accel=1/3, max_decel=2/3):
        """
        max_pwm: maximum PWM value
        min_pwm: minimum PWM value
        max_accel: maximum increase in PWM per second
        max_decel: maximum decrease in PWM per second
        """
        self.max_pwm = max_pwm
        self.min_pwm = min_pwm
        self.max_accel = max_accel
        self.max_decel = max_decel

        self.current_pwm = 0.0    # current PWM output

        # Simple proportional gain (tune as needed)
        self.kp = 1/70

    def update(self, dt):
        """
        Update control loop.

        dt: time step (seconds)

        Returns: new PWM value
        """

        # Proportional control to compute desired PWM
        error = state.target_speed - state.current_speed
        desired_pwm = self.current_pwm + self.kp * error

        # Clamp desired PWM to valid range
        desired_pwm = max(self.min_pwm, min(self.max_pwm, desired_pwm))

        # Apply acceleration/deceleration limits
        delta_pwm = desired_pwm - self.current_pwm

        if delta_pwm > 0:
            # Accelerating
            max_delta = self.max_accel * dt
            delta_pwm = min(delta_pwm, max_delta)
        else:
            # Decelerating
            max_delta = self.max_decel * dt
            delta_pwm = max(delta_pwm, -max_delta)

        # Update PWM
        self.current_pwm += delta_pwm

        # Final clamp (safety)
        self.current_pwm = max(self.min_pwm, min(self.max_pwm, self.current_pwm))

        if self.current_pwm > 0:
            Motor.frente()
        elif self.current_pwm == 0:
            Motor.parar()
        else:
            Motor.tras()

        Motor.ena.duty_u16(int(abs(self.current_pwm) * 65535))


cruiseControl = CruiseControl()

def _read_distance_sensors():
    Distance.get_sensor_data()

def _read_camera():
    Camera.update()

def _adjust_odometry():
    if state.clockwise == 0:
        return

    corrigir_corredor()
    corrigir_canto()

def corrigir_canto():
    curr_x, curr_y = state.get_relative_odom

    if curr_x > 90:
        return

    angle = state.compass_angle_relative

    if angle > 180:
        angle -= 360
    
    if abs(90 + angle * state.clockwise) > 10:
        return


    distance_left, distance_right, distance_front, distance_rear = get_odom_side_sonar()


    if distance_front > 80 or distance_front <= 6:
        return

    if state.clockwise == 1:
        if distance_left > 80 or distance_left <= 6:
            return

        state.set_relative_odom(distance_left + 2.55, distance_front)
    else:
        if distance_right > 80 or distance_right <= 6:
            return

        state.set_relative_odom(distance_right + 2.55, distance_front)


def corrigir_corredor():
    curr_x, curr_y = state.get_relative_odom

    distance_left, distance_right, distance_front, distance_rear = get_odom_side_sonar()

    if abs(state.compass_angle_relative) > 10:
        return

    #if curr_y > 50:    
    if curr_x < 100 or curr_x > 180:
        return
        
    #if curr_y < 5 or curr_y > 95:
    #    return

    if state.is_lane_with_parking:
        if curr_y < 62:
            if state.clockwise == 1 and curr_x < 150:
                return
            if state.clockwise == -1 and curr_x > 150:
                return
            

    if curr_y < 50 and state.clockwise == 1 or curr_y > 50 and state.clockwise == -1:
        if distance_left > 50 or distance_left <= 6:
            return

        offset = distance_left + 2.55


        state.set_relative_odom(state.get_relative_odom[0], abs((100 if curr_y > 50 else 0) - offset))
    else:
        if distance_right > 50 or distance_right <= 6:
            return

        offset = distance_right + 2.55

        state.set_relative_odom(state.get_relative_odom[0], abs((100 if curr_y > 50 else 0) - offset))

def get_abs_distance(sonar_offset, angle, distance, is_side = False):

    hipotenusa = math.sqrt(sonar_offset[0]**2 + (sonar_offset[1]+distance)**2) if is_side else distance + sonar_offset[0]

    angle1 =  math.radians(90 - angle) - (math.atan2(sonar_offset[1]+distance, sonar_offset[0]) if is_side else 0)
    
    y = hipotenusa * math.cos(angle1)


    return y

def get_odom_side_sonar():
    sonar_interval = 1/10
    distance_delta = sonar_interval * state.current_speed
    side_sonar_offset = (7.97, 2.55)
    front_sonar_offset = (8.97, 0)
    rear_sonar_offset = (1.25, 0)

    angle = state.compass_angle_relative

    while angle > 45:
        angle -= 90

    while angle < -45:
        angle += 90

    angle_rad = math.radians(angle)
    side_distance_backtrack = math.tan(angle_rad) * distance_delta


    is_positive_angle = angle > 0

    if not is_positive_angle:
        side_distance_backtrack = -side_distance_backtrack

    distance_left = get_abs_distance(side_sonar_offset, -angle, Distance.get_left() + side_distance_backtrack, True)
    distance_right = get_abs_distance(side_sonar_offset, angle, Distance.get_right() - side_distance_backtrack, True)
    distance_front = get_abs_distance(front_sonar_offset, 90-angle, Distance.get_front() - distance_delta)
    distance_rear = get_abs_distance(rear_sonar_offset, 90+angle, Distance.get_rear() + distance_delta)

    return distance_left, distance_right, distance_front, distance_rear


def _update_odometry(last_distance_cm):
    compass_angle = Compass.heading()
    state.compass_angle = compass_angle

    current_distance_cm = Encoder.distance_cm()
    delta_cm = current_distance_cm - last_distance_cm
    theta_rad = math.radians(compass_angle)

    state.odom_x += delta_cm * math.cos(theta_rad) * state.clockwise * -1
    state.odom_y += delta_cm * math.sin(theta_rad) * state.clockwise

    return current_distance_cm


def update_speed(last_distance_cm, last_speed_ms):
    current_distance_cm = Encoder.distance_cm()
    delta_cm = current_distance_cm - last_distance_cm
    delta_time_ms = time.ticks_diff(time.ticks_ms(), last_speed_ms)
    delta_time_s = delta_time_ms / 1000.0

    if delta_time_ms <= 0:
        return current_distance_cm

    state.current_speed = delta_cm / delta_time_s

    cruiseControl.update(delta_time_s)

    return current_distance_cm

def _send_telemetry():
    """Transmit current pose and distance sensor readings."""
    Telemetry.send()


def background_task():
    last_distance_cm = Encoder.distance_cm()
    last_speed_cm = last_distance_cm
    last_sensor_ms = time.ticks_ms()
    last_camera_ms = last_sensor_ms
    last_odom_ms = last_sensor_ms
    last_speed_ms = last_sensor_ms
    last_telemetry_ms = last_sensor_ms
    last_adjust_odom_ms = last_sensor_ms

    print("Background task started")

    while True:
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_sensor_ms) >= SENSOR_INTERVAL_MS:
            _read_distance_sensors()
            last_sensor_ms = now_ms

        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_camera_ms) >= CAMERA_INTERVAL_MS:
            _read_camera()
            last_camera_ms = now_ms

        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_odom_ms) >= ODOM_INTERVAL_MS:
            last_distance_cm = _update_odometry(last_distance_cm)
            last_odom_ms = now_ms

        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_speed_ms) >= SPEED_INTERVAL_MS:
            last_speed_cm = update_speed(last_speed_cm, last_speed_ms)
            last_speed_ms = now_ms

        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_telemetry_ms) >= TELEMETRY_INTERVAL_MS:
            _send_telemetry()
            last_telemetry_ms = now_ms

        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_adjust_odom_ms) >= ADJUST_ODOM_INTERVAL_MS:
            _adjust_odometry()
            last_adjust_odom_ms = now_ms

        time.sleep_ms(LOOP_SLEEP_MS)
        