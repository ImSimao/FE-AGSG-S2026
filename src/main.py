import init
from obstacle_challenge import obstacle_challenge
from state import state
from servo import Servo
from motor import Motor


def main():
    try:
        obstacle_challenge()
    finally:
        state.target_speed = 0
        Motor.parar()
        Motor.ena.duty_u16(0)
        Servo.set_angle(0)


if __name__ == "__main__":
    main()
