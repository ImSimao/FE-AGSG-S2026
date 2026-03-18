import time
from compass import Compass
import motor
from state import state
from distance import Distance
from motor import Motor
from servo import Servo
import math
from pid import PIDController


def get_angle_to_rotate(dest_x, dest_y):
    x_initial  = state.relative_odom_x
    y_initial = state.relative_odom_y

    # Coordenadas absolutas
    A = (x_initial, y_initial)
    B = (dest_x, dest_y)

    compass_angle = state.compass_angle

    if compass_angle > 180:
        compass_angle -= 360

    angle_to_coord = math.degrees(math.atan2(dest_y - y_initial, dest_x - x_initial))


    angle_to_rotate = angle_to_coord - compass_angle

    if angle_to_rotate > 180:
        angle_to_rotate -= 360

    print("angle_to_rotate: ", angle_to_rotate)
    print("compass_angle: ", compass_angle)
    print("angle_to_coord: ", angle_to_coord)

    return angle_to_rotate

def rotate_coordinates(dest_x, dest_y):
    while True:
        angle_to_rotate = get_angle_to_rotate(dest_x, dest_y)

        if abs(angle_to_rotate) < 1.5:
            Servo.set_angle(0)
            Motor.parar()
            break

        Motor.frente(0.35)

        if angle_to_rotate > 0:
            Servo.set_angle(60)
        else:
            Servo.set_angle(-60)

        time.sleep(1/20)


def move_coordinates(dest_x, dest_y):

    rotate_coordinates(dest_x, dest_y)

    x_initial  = state.relative_odom_x
    y_initial = state.relative_odom_y

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
        C = (state.relative_odom_x, state.relative_odom_y)

        # Vetor AC
        ACx = C[0] - A[0]
        ACy = C[1] - A[1]

        # Coordenadas no sistema rotacionado
        x_prime = ACx*ux + ACy*uy
        y_prime = ACx*ux_perp + ACy*uy_perp

        
        if L - x_prime < 0:
            Motor.parar()
            pid_y.reset()
            Servo.set_angle(0)
            break
        
        Motor.frente(0.7)

        servo_angle = pid_y.compute(-y_prime, dt) 
        Servo.set_angle(servo_angle)

        time.sleep(dt)
