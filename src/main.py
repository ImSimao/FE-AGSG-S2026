import init
from move_coordinates import move_coordinates
from obstacle_challenge import obstacle_challenge
from state import state
from servo import Servo
from motor import Motor


def main():
    try:
        state.clockwise = -1
        #obstacle_challenge()
        move_coordinates(100, 0)
    finally:
        state.target_speed = 0
        Motor.stop()
        Motor.ena.duty_u16(0)
        Servo.set_angle(0)


if __name__ == "__main__":
    main()
