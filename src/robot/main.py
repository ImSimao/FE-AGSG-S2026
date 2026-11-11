import init
from obstacle_challenge import obstacle_challenge
from open_challenge import open_challenge
from state import state
from servo import Servo
from motor import Motor
import test_pid


def main():
    try:
        #test_pid.test_pid()
        obstacle_challenge()
        #open_challenge()
    finally:
        state.target_speed = 0
        Motor.stop()
        Motor.ena.duty_u16(0)
        Servo.set_angle(0)


if __name__ == "__main__":
    main()
