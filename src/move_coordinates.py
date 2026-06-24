import time
from state import state
from servo import Servo
import math
from pid import PIDController
from telemetry import Telemetry
from math import sin, cos


def get_angle_to_rotate(dest_x, dest_y):
    x_initial, y_initial = state.get_relative_odom

    # Coordenadas absolutas
    A = (x_initial, y_initial)
    B = (dest_x, dest_y)

    compass_angle = state.compass_angle_relative

    if compass_angle > 180:
        compass_angle -= 360

    # Odom: dy = sin(heading)*clockwise — for clockwise=-1, forward in relative
    # frame matches atan2 bearing -heading, not +heading.
    if state.clockwise == -1:
        compass_angle = -compass_angle

    angle_to_coord = math.degrees(math.atan2(dest_y - y_initial, dest_x - x_initial))

    angle_to_rotate = angle_to_coord - compass_angle

    if angle_to_rotate > 180:
        angle_to_rotate -= 360

    if angle_to_rotate < -180:
        angle_to_rotate += 360

    return angle_to_rotate

def rotate_coordinates(dest_x, dest_y, reverse = False):
    initial = True

    if target_in_rotation_area(dest_x, dest_y):
        reverse = not reverse
        #return

    while True:
        angle_to_rotate = get_angle_to_rotate(dest_x, dest_y)

        if abs(angle_to_rotate) < 8:
            Servo.set_angle(0)
            state.target_speed = 0
            break

        state.target_speed = 15

        servo_angle = Servo._MAX_STEERING_OFFSET * state.clockwise

        if reverse:
            servo_angle = -servo_angle
            state.target_speed = -state.target_speed

        if angle_to_rotate > 0:
            Servo.set_angle(servo_angle)
        else:
            Servo.set_angle(-servo_angle)

        if initial:
            initial = False
            time.sleep(0.5)

        time.sleep(1/100)


def rotate_angle(angle, reverse = False):
    while True:
        angle_to_rotate = angle - state.compass_angle_relative

        if angle_to_rotate > 180:
            angle_to_rotate -= 360

        if angle_to_rotate < -180:
            angle_to_rotate += 360

        if abs(angle_to_rotate) < 2:
            Servo.set_angle(0)
            state.target_speed = 0
            break

        state.target_speed = 15

        servo_angle = Servo._MAX_STEERING_OFFSET

        if reverse:
            servo_angle = -servo_angle
            state.target_speed = -state.target_speed

        if angle_to_rotate > 0:
            Servo.set_angle(servo_angle)
        else:
            Servo.set_angle(-servo_angle)


        time.sleep(1/100)

def move_coordinates(dest_x, dest_y, reverse = False, rotate = True):

    desaccelerate_distance = 40
    max_speed = 60
    min_speed = 15

    x_initial, y_initial = state.get_relative_odom

    if rotate:
        if reverse:
            rotate_coordinates(x_initial- (dest_x - x_initial), y_initial- (dest_y - y_initial))
        else:
            rotate_coordinates(dest_x, dest_y)


    x_initial, y_initial = state.get_relative_odom

    # Coordenadas absolutas
    A = (x_initial, y_initial)
    B = (dest_x, dest_y)

    # Vetor AB
    ABx = B[0] - A[0]
    ABy = B[1] - A[1]

    # Comprimento de AB
    L = math.sqrt(ABx*ABx + ABy*ABy)

    # Vetor unitário na direção AB (eixo x')
    ux = ABx / L
    uy = ABy / L

    # Vetor perpendicular (eixo y')
    ux_perp = -uy
    uy_perp = ux

    pid_y = PIDController(kp=0.35, ki=0.003, kd=0.5)
    dt = 1/100

    while True:
        C = state.get_relative_odom

        # Vetor AC
        ACx = C[0] - A[0]
        ACy = C[1] - A[1]

        # Coordenadas no sistema rotacionado
        x_prime = ACx*ux + ACy*uy
        y_prime = ACx*ux_perp + ACy*uy_perp
        remaining_distance = L - x_prime

        
        if remaining_distance < 0:
            state.target_speed = 0
            pid_y.reset()
            Servo.set_angle(0)
            break

        if remaining_distance < desaccelerate_distance:
            state.target_speed = remaining_distance / desaccelerate_distance * (max_speed - min_speed) + min_speed
        else:
            state.target_speed = max_speed

        servo_angle = pid_y.compute(-y_prime, dt) 

        if reverse:
            state.target_speed = -state.target_speed
            servo_angle = -servo_angle



        Servo.set_angle(servo_angle*state.clockwise)

        time.sleep(dt)


def circle_center_left(xr, yr, theta, radius):
    xc = xr - radius * sin(theta)
    yc = yr + radius * cos(theta)
    return xc, yc

def circle_center_right(xr, yr, theta, radius):
    xc = xr + radius * sin(theta)
    yc = yr - radius * cos(theta)
    return xc, yc

def is_inside_circle(xt, yt, xc, yc, radius):
    dx = xt - xc
    dy = yt - yc
    return (dx * dx + dy * dy) <= (radius * radius)

def target_in_left_rotation_area(xr, yr, theta, xt, yt, radius):
    xc, yc = circle_center_left(xr, yr, theta, radius)
    return is_inside_circle(xt, yt, xc, yc, radius)

def target_in_right_rotation_area(xr, yr, theta, xt, yt, radius):
    xc, yc = circle_center_right(xr, yr, theta, radius)
    return is_inside_circle(xt, yt, xc, yc, radius)

def target_in_rotation_area(xt, yt, radius=30):
    xr, yr = state.get_relative_odom
    theta = math.radians(state.compass_angle_relative)

    left = target_in_left_rotation_area(xr, yr, theta, xt, yt, radius)
    right = target_in_right_rotation_area(xr, yr, theta, xt, yt, radius)
    return left or right