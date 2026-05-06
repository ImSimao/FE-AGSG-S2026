import time
from background import get_odom_side_sonar
from  move_coordinates import move_coordinates, rotate_angle, rotate_coordinates
from distance import Distance
from state import state

#Coordenadas




ninety_degrees_distance_offset = 18

traffic_lane_center = (state.parede_fora - state.parede_dentro) / 4
traffic_lane_y_offset = 32
traffic_lane_y_parking = 11.5
firt_obstacle_camera_coord = (100, traffic_lane_center)
second_obstacle_camera_coord = (175, traffic_lane_center)
first_traffic_lane_camera_x = 100
final_traffic_lane_x = state.parede_fora - traffic_lane_center - traffic_lane_y_offset
parking_x = 123
parking_y = 25


def get_traffic_lane_y(traffic_inside, rotate = False, reverse = False):
        return traffic_lane_center \
        + ((traffic_lane_y_offset * traffic_inside) if not state.is_lane_with_parking or traffic_inside == 1 \
            else - traffic_lane_y_parking ) \
             - (ninety_degrees_distance_offset * traffic_inside if rotate else 0)
 


def obstacle_challenge():
    time.sleep(1)

    if Distance.get_left() < Distance.get_right() and Distance.get_left() > 0 or Distance.get_right() <= 0:
        state.clockwise = 1
    else:
        state.clockwise = -1

    distance_left, distance_right, distance_front, distance_rear = get_odom_side_sonar()

    if state.clockwise == 1:
        state.set_relative_odom(100+1.75, distance_left)
        rotate_coordinates(200, 100)
    else:
        state.set_relative_odom(200-distance_front, distance_right)
        rotate_coordinates(200, 60)

    traffic_inside = -1

    if traffic_inside == 1:
        move_coordinates(state.get_relative_odom[0], get_traffic_lane_y(traffic_inside, True))
    else:
        rotate_angle(65*state.clockwise)
    
    move_coordinates(230, get_traffic_lane_y(traffic_inside))
    
    voltas = 1

    while state.lap <= voltas:
        state.current_lane += 1

        is_same_color = False
        traffic_inside = -1

        if state.lap == 0:
            rotate_coordinates(firt_obstacle_camera_coord[0], firt_obstacle_camera_coord[1])
        
            if traffic_inside == 1:
                rotate_angle(0, reverse=True)
            else:
                rotate_angle(-90*state.clockwise, reverse=True)
                move_coordinates(state.get_relative_odom[0], get_traffic_lane_y(traffic_inside, True))
                rotate_angle(0)

        move_coordinates(first_traffic_lane_camera_x, get_traffic_lane_y(traffic_inside))

        if state.lap == voltas:
            break

        traffic_inside = 1

        if state.lap == 0:
            rotate_coordinates(second_obstacle_camera_coord[0], second_obstacle_camera_coord[1])

            if is_same_color:
                rotate_angle(0, reverse=True)
        
        if not is_same_color:
            rotate_angle(90*traffic_inside*state.clockwise)
            move_coordinates(state.get_relative_odom[0], get_traffic_lane_y(traffic_inside, True))
            rotate_angle(0)

        final_traffic_lane_x1 = final_traffic_lane_x

        if traffic_inside == 1:
            final_traffic_lane_x1 += ninety_degrees_distance_offset
        else:
            final_traffic_lane_x1 -= ninety_degrees_distance_offset

        move_coordinates(final_traffic_lane_x1, get_traffic_lane_y(traffic_inside))


    #Parking
    if traffic_inside == 1:
        move_coordinates(state.get_relative_odom[0], parking_y)
    
    rotate_angle(0, reverse=True)
    move_coordinates(parking_x, state.get_relative_odom[1])
    rotate_angle(90 * state.clockwise, reverse=True)
    rotate_angle(0, reverse=True)
        



