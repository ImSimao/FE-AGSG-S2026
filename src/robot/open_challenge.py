import time
import move_coordinates
from distance import Distance
from state import state


def open_challenge():

    state.clockwise = -1

    move_coordinates.move_coordinates(80, 0)

    while Distance.get_front() > 45:
        move_coordinates.move_coordinates(state.odom_x + 20, state.odom_y)

    move_coordinates.rotate_angle(0)

    time.sleep(1)

    while True:

        clockwise = 0

        l = Distance.get_left()
        r = Distance.get_right()


        if l > 0 and l < 90:
            clockwise = 1

        if r > 0 and r < 90:
            clockwise = -1


        if clockwise != 0:
            state.clockwise = clockwise
            break

        time.sleep(0.1)


    if state.clockwise == 1:
        state.set_relative_odom(300-Distance.get_front()-9, Distance.get_left()+3)
    else:
        state.set_relative_odom(300-Distance.get_front()-9, Distance.get_right()+3)

    state.current_lane = 1

    while state.current_lane < 12:
        move_coordinates.move_coordinates(247, 32)
        state.current_lane += 1

    move_coordinates.move_coordinates(150, 32)
