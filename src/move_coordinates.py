import time
from compass import Compass
import motor
from state import state
from distance import Distance
from motor import Motor
from servo import Servo
import math

def rotate_coordinates(dest_x, dest_y):

    while True:
        x_initial  = state.relative_odom_x
        y_initial = state.relative_odom_y

        # Coordenadas absolutas
        A = (x_initial, y_initial)
        B = (dest_x, dest_y)

        compass_angle = state.compass_angle

        angle_to_coord = math.degrees(math.atan2(dest_y - y_initial, dest_x - x_initial))

        if angle_to_coord < 0:
            angle_to_coord += 360

        angle_to_rotate = angle_to_coord - compass_angle

        if abs(angle_to_rotate) < 5:
            Motor.parar()
            Servo.set_angle(0)
            break 

        if angle_to_rotate > 180:
            angle_to_rotate = angle_to_rotate - 360

        Motor.frente(0.2)

        if angle_to_rotate > 0:
            Servo.set_angle(40)
        else:
            Servo.set_angle(-40)

        time.sleep(0.1)

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
            Servo.set_angle(0)
            break
        
        Motor.frente(0.2)

        servo_angle = y_prime * 6.0
        servo_angle = servo_angle * -1
        
        Servo.set_angle(servo_angle)

        time.sleep(0.1)
