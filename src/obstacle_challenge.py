import time
from background import get_odom_side_sonar
from field import Field, Lane
from  move_coordinates import move_coordinates, rotate_angle, rotate_coordinates
from distance import Distance
from state import State, state
from telemetry import Telemetry

#Coordenadas




ninety_degrees_distance_offset = 18

traffic_lane_center = (state.parede_fora - state.parede_dentro) / 4
traffic_lane_y_offset = 30    #32
traffic_lane_y_parking = 11.5
firt_obstacle_camera_coord = (100, traffic_lane_center)
second_obstacle_camera_coord = (200, traffic_lane_center)
first_traffic_lane_camera_x = 150
final_traffic_lane_x = state.parede_fora - traffic_lane_center - traffic_lane_y_offset
parking_x_offset = 23
parking_gap = 30
parking_y = 30


def get_traffic_lane_y(traffic_side, rotate = False, reverse = False):
        return traffic_lane_center \
        + ((traffic_lane_y_offset * -traffic_side) if not state.is_lane_with_parking or traffic_side == -1 \
            else - traffic_lane_y_parking ) \
             - (ninety_degrees_distance_offset * -traffic_side if rotate else 0)
 


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

    if not Field.parking_traffic_exit_confirmation():
        a=0
        #procurar cor
        print ("Não detetou") 

    if Field.lanes_sides()[1] == -1:
        move_coordinates(state.get_relative_odom[0], get_traffic_lane_y(Field.lanes_sides()[1], True))
    else:
        rotate_angle(65*state.clockwise)
    
    move_coordinates((State.parede_dentro/2) + (State.parede_fora/2) + 4, get_traffic_lane_y(Field.lanes_sides()[1]), rotate = False)
    Field.exiting_park = False
    
    voltas = 3

    while state.lap <= voltas:
        state.current_lane += 1

    
        if not Field.can_enter_lane():
            #procurar cor
            if state.is_lane_with_parking:
                move_coordinates(50, 45)
            else:
                move_coordinates(50, 35)
            
            rotate_angle(0, reverse=True)
                
            rotate_coordinates(firt_obstacle_camera_coord[0], firt_obstacle_camera_coord[1] if not state.is_lane_with_parking else 60)
            
            while not Field.can_enter_lane():
                move_coordinates(state.get_relative_odom[0] + 10, 50 if not state.is_lane_with_parking else 60, rotate=False)
                
                if state.get_relative_odom[0] > 100:
                    break
            
            rotate_angle(-90*Field.lanes_sides()[0]*state.clockwise, True)
            move_coordinates(state.get_relative_odom[0], get_traffic_lane_y(Field.lanes_sides()[0], True), rotate=False)

        elif Field.lanes_sides()[0] == 1:
            move_coordinates(state.get_relative_odom[0], get_traffic_lane_y(Field.lanes_sides()[0], True), rotate=False)
    
        rotate_angle(0)
        move_coordinates(first_traffic_lane_camera_x, get_traffic_lane_y(Field.lanes_sides()[0]), rotate = False)

        if state.lap == voltas:
            break


        if not Field.can_exit_lane():
            rotate_coordinates(second_obstacle_camera_coord[0], second_obstacle_camera_coord[1])
            time.sleep(1)

            sides = Field.lanes_sides()
            if sides[0] == sides[1]:
                rotate_angle(0, reverse=True)
        
        if Field.lanes_sides()[1] == -1 and state.get_relative_odom[1] < 50 or Field.lanes_sides()[1] == 1 and state.get_relative_odom[1] > 50:
            rotate_angle(-90*Field.lanes_sides()[1]*state.clockwise)
            move_coordinates(state.get_relative_odom[0], get_traffic_lane_y(Field.lanes_sides()[1], True))
            rotate_angle(0)

        move_coordinates((State.parede_dentro/2) + (State.parede_fora/2) + 4, get_traffic_lane_y(Field.lanes_sides()[1]), rotate = False)


    #Parking
    if state.clockwise == -1 and Field.lanes_sides()[0] == -1:
        move_coordinates(160, get_traffic_lane_y(Field.lanes_sides()[0]))

    if Field.lanes_sides()[0] == -1:
        rotate_angle(-90 * state.clockwise)
        move_coordinates(state.get_relative_odom[0], parking_y)
    
    rotate_angle(0, reverse=True)
    move_coordinates((100 + parking_x_offset) if state.clockwise == 1 else 190 - parking_gap + parking_x_offset, state.get_relative_odom[1])
    rotate_angle(90 * state.clockwise, reverse=True)
    rotate_angle(0, reverse=True)
        



