import math
from state import state

camera_angle = 70
camera_distance = 50
robot_distance_from_camera = 9


def get_colour_position(traffic_x, traffic_y):
    camera_width = 160
    camera_height = 120
    camera_center_x = camera_width / 2
    camera_center_y = camera_height

    max_angle_difference = 2


    dx = traffic_x - camera_center_x
    dy = traffic_y - camera_center_y

    distance = math.sqrt(dx**2 + dy**2)
    relative_angle = math.degrees(math.atan2(dy, dx))
    relative_angle = (relative_angle + 180) % 360 - 180

    possible_traffic_positions = get_traffic_lane_inside_pov()

    for position in possible_traffic_positions:
        if abs(position[3] - relative_angle) <= max_angle_difference:
            # guardar na memoria
    return None




    


def get_traffic_lane_inside_pov():
    traffic_coord = [
        # Bottom Lane
        [100, 60, 0],
        [150, 60, 1],
        [200, 60, 1],
        [240, 100, 2]
    ];

    if not state.is_lane_with_parking:
        traffic_coord.append([100, 40, 0])
        traffic_coord.append([150, 40, 1])
        traffic_coord.append([200, 40, 1])

    elif state.relative_lane != 3:
        traffic_coord.append([260, 100, 2])

    x, y = state.get_relative_odom
    x_camera = x + robot_distance_from_camera * math.sen(state.compass_angle_relative)
    y_camera = y + robot_distance_from_camera * math.cos(state.compass_angle_relative)

    position_inside_pov = []
    
    for lane in traffic_coord:
        dx = lane[0] - x_camera
        dy = lane[1] - y_camera
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance <= camera_distance:
           
            relative_angle = math.degrees(math.atan2(dy, dx)) - state.compass_angle_relative
            relative_angle = (relative_angle + 180) % 360 - 180

            if abs(relative_angle) <= camera_angle / 2:
                position_inside_pov.append([lane[0], lane[1], lane[2], relative_angle, distance])
    return position_inside_pov

