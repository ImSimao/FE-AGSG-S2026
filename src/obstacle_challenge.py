import time
import uasyncio as asyncio
from background import get_odom_side_sonar
from field import Field, Lane
from  move_coordinates import move_coordinates, rotate_angle, rotate_coordinates
from distance import Distance
from state import State, state
from telemetry import Telemetry

#Coordenadas




ninety_degrees_distance_offset =18 # 16.5

traffic_lane_center = (state.parede_fora - state.parede_dentro) / 4
traffic_lane_y_offset = 30 #26.25
traffic_lane_y_parking = 11.5
firt_obstacle_camera_coord = (100, traffic_lane_center)
second_obstacle_camera_coord = (200, traffic_lane_center)
first_traffic_lane_camera_x = 150
final_traffic_lane_x = state.parede_fora - traffic_lane_center - traffic_lane_y_offset
parking_x_offset = 5
parking_gap = 45
parking_y = 29


def get_traffic_lane_y(traffic_side, rotate = False, reverse = False):
        return traffic_lane_center \
        + ((traffic_lane_y_offset * -traffic_side) if not state.is_lane_with_parking or traffic_side == -1 \
            else - traffic_lane_y_parking ) \
             - (ninety_degrees_distance_offset * -traffic_side if rotate else 0)
 



async def obstacle_challenge():
    global first_traffic_lane_camera_x
    await asyncio.sleep(1)

    if Distance.get_left() < Distance.get_right() and (Distance.get_left() > 0 or Distance.get_right() <= 0):
        state.clockwise = 1
    else:
        state.clockwise = -1

    distance_left, distance_right, distance_front, distance_rear = get_odom_side_sonar()

    if state.clockwise == 1:
        state.set_relative_odom(100+1.75, distance_left)
        await rotate_coordinates(200, 100)
        await asyncio.sleep(1)
    else:
        state.set_relative_odom(200-distance_front, distance_right)
        await rotate_coordinates(200, 60)
        await asyncio.sleep(1)

    if not Field.parking_traffic_exit_confirmation():
        a=0
        #procurar cor
        print ("Não detetou") 

    if Field.lanes_sides()[1] == -1:
        await move_coordinates(state.get_relative_odom[0], get_traffic_lane_y(Field.lanes_sides()[1], True))
    else:
        await rotate_angle(65*state.clockwise)
    
    await rotate_angle(0)
    await move_coordinates((State.parede_dentro/2) + (State.parede_fora/2), get_traffic_lane_y(Field.lanes_sides()[1]), rotate = False)
    Field.exiting_park = False
    
    voltas = 3

    while state.lap <= voltas:
        state.current_lane += 1

    
        if not Field.can_enter_lane():
            #procurar cor
            if state.is_lane_with_parking:
                await move_coordinates(50, 45)
            else:
                await move_coordinates(50, 35)
            
            await rotate_angle(0, reverse=True)
                
            await rotate_coordinates(firt_obstacle_camera_coord[0], firt_obstacle_camera_coord[1] if not state.is_lane_with_parking else 60)
            
            while not Field.can_enter_lane():
                await move_coordinates(state.get_relative_odom[0] + 10, 50 if not state.is_lane_with_parking else 60, rotate=False)
                
                if state.get_relative_odom[0] > 100:
                    break
            
            await rotate_angle(-90*Field.lanes_sides()[0]*state.clockwise, True)
            await move_coordinates(state.get_relative_odom[0], get_traffic_lane_y(Field.lanes_sides()[0], True), rotate=False)

        elif Field.lanes_sides()[0] == 1:
            await move_coordinates(state.get_relative_odom[0], get_traffic_lane_y(Field.lanes_sides()[0], True), rotate=False)

        elif Field.lanes_sides()[0] == -1:
            if state.get_relative_odom[0] > 50:
                await move_coordinates(state.get_relative_odom[0], 65, rotate=False)
                await rotate_angle(0, reverse=True)
    
    
        await rotate_angle(0)
        
        if state.lap == voltas - 1:
            if state.clockwise == 1:
                first_traffic_lane_camera_x = 95
            else:
                first_traffic_lane_camera_x = 175

        if state.lap == voltas:
            break

        await move_coordinates(first_traffic_lane_camera_x, get_traffic_lane_y(Field.lanes_sides()[0]), rotate = False)

        if not Field.can_exit_lane():
            await rotate_coordinates(second_obstacle_camera_coord[0], second_obstacle_camera_coord[1])
            
            start_time = time.ticks_ms()
            while not Field.can_exit_lane():
                await asyncio.sleep(0.1)

                if time.ticks_ms() - start_time > 3000:
                    break

            sides = Field.lanes_sides()
            if sides[0] == sides[1]:
                await rotate_angle(0, reverse=True)
        
        if Field.lanes_sides()[1] == -1 and state.get_relative_odom[1] < 50 or Field.lanes_sides()[1] == 1 and state.get_relative_odom[1] > 50:
            await rotate_angle(-90*Field.lanes_sides()[1]*state.clockwise)
            await move_coordinates(state.get_relative_odom[0], get_traffic_lane_y(Field.lanes_sides()[1], True))
            await rotate_angle(0)

        await move_coordinates((State.parede_dentro/2) + (State.parede_fora/2), get_traffic_lane_y(Field.lanes_sides()[1]), rotate = False)


    #parking      
    await move_coordinates((100 + parking_x_offset) if state.clockwise == 1 else 200 - parking_gap + parking_x_offset, state.get_relative_odom[1], rotate = False)
    
    await rotate_angle(-90 * state.clockwise)
    
    if state.get_relative_odom[1] > 50:
        await move_coordinates(state.get_relative_odom[0], parking_y, rotate=False)
        await rotate_angle(-90 * state.clockwise)

    await asyncio.sleep(1)

    parking_start_ms = time.ticks_ms()
    while True:
        if time.ticks_diff(time.ticks_ms(), parking_start_ms) > 10000:
            state.target_speed = 0
            break

        diff_distance = Distance.get_front() - 18

        if abs(diff_distance) < 1:
            state.target_speed = 0

            if Distance.get_left() < 30 and Distance.get_right() < 30:
                if Distance.get_left() < Distance.get_right():
                    await rotate_angle(-90 * state.clockwise + 25, reverse=True)
                else:
                    await rotate_angle(-90 * state.clockwise - 25, reverse=True)
                await rotate_angle(-90 * state.clockwise, reverse=True)
                await asyncio.sleep(0.5)
            else:
                break

        if diff_distance > 0:
            state.target_speed = 10
        else:
            state.target_speed = -10

        await asyncio.sleep_ms(50)

    await asyncio.sleep(1)

    if Distance.get_left() < Distance.get_right():
        await rotate_angle(0 if state.clockwise == 1 else 180)
    else:
        await rotate_angle(0 if state.clockwise == -1 else 180)

    await rotate_angle(0 if abs(state.compass_angle_relative - 180) > 90 else 180, reverse=True)

    parking_start_ms = time.ticks_ms()
    while True:
        if time.ticks_diff(time.ticks_ms(), parking_start_ms) > 5000:
            state.target_speed = 0
            break

        diff_distance = Distance.get_rear() - 7.5

        if abs(diff_distance) < 1:
            state.target_speed = 0
            break

        if diff_distance > 0:
            state.target_speed = -10
        else:
            state.target_speed = 10

        await asyncio.sleep_ms(50)
        
    await rotate_angle(0 if abs(state.compass_angle_relative - 180) > 90 else 180)

    await asyncio.sleep(1)
    Telemetry.log(state.compass_angle_relative)
