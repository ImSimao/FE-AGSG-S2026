import time
from  move_coordinates import move_coordinates, rotate_angle, rotate_coordinates
from distance import Distance
from state import state


def obstacle_challenge():
    time.sleep(1)

    if Distance.get_left() < Distance.get_right() and Distance.get_left() > 0 or Distance.get_right() <= 0:
        state.clockwise = 1
    else:
        state.clockwise = -1

    if state.clockwise == 1:
        state.set_relative_odom(100+2, Distance.get_left()+3)

        rotate_coordinates(200, 100)

        traffise_inside = True

        if traffise_inside == True:
            move_coordinates(125, 75)
            move_coordinates(230, state.get_relative_odom[1])
    else:
        state.set_relative_odom(200-Distance.get_front()-9, Distance.get_right()+3)

    voltas = 1

    while state.lap <= voltas:
        state.current_lane += 1
        rotate_coordinates(100, 50, reverse=True)
        rotate_angle(0, reverse=True)


        if traffise_inside == True:
            move_coordinates(115, 80)

            if state.lap == voltas:
                break

            rotate_coordinates(175, 50)

            if traffise_inside == True:
                rotate_angle(0, reverse=True)
                move_coordinates(230, 70)
                rotate_angle(90 * state.clockwise, reverse=True)
            else:
                move_coordinates(120, 35)
                move_coordinates(210, 25)


    #Parking
    if traffise_inside == True:
        time.sleep(2)
        move_coordinates(110, 25)
        time.sleep(2)
        rotate_angle(0, reverse=True)
        time.sleep(2)
        move_coordinates(120, state.get_relative_odom[1])
        time.sleep(2)
        rotate_angle(90 * state.clockwise, reverse=True)
        time.sleep(2)
        #move_coordinates(state.get_relative_odom[0], 40, reverse=True)
        #time.sleep(2)
        rotate_angle(0, reverse=True)
        



