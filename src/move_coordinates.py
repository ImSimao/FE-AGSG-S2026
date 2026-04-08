import time
from state import state
from servo import Servo
import math
from pid import PIDController
from telemetry import Telemetry


def get_angle_to_rotate(dest_x, dest_y):
    x_initial, y_initial = state.get_relative_odom

    # Coordenadas absolutas
    A = (x_initial, y_initial)
    B = (dest_x, dest_y)

    compass_angle = state.compass_angle_relative

    if compass_angle > 180:
        compass_angle -= 360

    angle_to_coord = math.degrees(math.atan2(dest_y - y_initial, dest_x - x_initial))

    angle_to_rotate = angle_to_coord - compass_angle

    if angle_to_rotate > 180:
        angle_to_rotate -= 360

    if angle_to_rotate < -180:
        angle_to_rotate += 360

    return angle_to_rotate

def rotate_coordinates(dest_x, dest_y):
    initial = True

    while True:
        angle_to_rotate = get_angle_to_rotate(dest_x, dest_y)

        if abs(angle_to_rotate) < 1.5:
            Servo.set_angle(0)
            state.target_speed = 0
            break

        state.target_speed = 13

        servo_angle = 42

        if angle_to_rotate > 0:
            Servo.set_angle(servo_angle)
        else:
            Servo.set_angle(-servo_angle)

        if initial:
            initial = False
            time.sleep(1)

        time.sleep(1/20)


def move_coordinates(dest_x, dest_y):
    desaccelerate_distance = 40
    max_speed = 70
    min_speed = 5

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

    pid_y = PIDController(kp=0.35, ki=0.0, kd=1.6)
    dt = 1/60

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

        #if state.clockwise == 1 and state.relative_lane != 2:
        #    servo_angle = -servo_angle

        Servo.set_angle(servo_angle)

        time.sleep(dt)
