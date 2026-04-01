import init
import time
import move_coordinates
from servo import Servo
from distance import Distance
from state import state


def open_challenge():
    move_coordinates.move_coordinates(100, 0)


    move_coordinates.move_coordinates(100 + Distance.get_front() - 15, 0)

    time.sleep(10)

    if Distance.get_left() < Distance.get_right() and Distance.get_left() > 0 or Distance.get_right() <= 0:
        state.clockwise = 1
    else:
        state.clockwise = -1

    if state.clockwise == 1:
        state.set_relative_odom(300-Distance.get_front()-9, Distance.get_left()+3)
    else:
        state.set_relative_odom(300-Distance.get_front()-9, Distance.get_right()+3)

    state.current_lane = 1

    while state.current_lane < 12:
        move_coordinates.move_coordinates(270, 30)
        state.current_lane += 1

    move_coordinates.move_coordinates(150, 50)

