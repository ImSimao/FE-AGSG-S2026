import math
from field import Field
from machine import UART, Pin
from state import state

camera_angle = 70
camera_distance = 90
robot_distance_from_camera = 13.5


class Camera:
    """
    Static helper for reading traffic-light detections from the OpenMV camera.

    Expects newline-terminated lines from the camera UART in the form:

        CAM:cx,bottom_y,COLOR;cx,bottom_y,COLOR

    Empty frames are sent as ``CAM:``.
    """

    _uart = None
    _rx_buffer = b""
    _last_blobs = []

    @staticmethod
    def initialize(uart_id=1, baudrate=115200, rx_pin=9):
        try:
            Camera._uart = UART(uart_id, baudrate=baudrate, rx=Pin(rx_pin))
            Camera._uart.read()
        except Exception:
            Camera._uart = None
            raise

    @staticmethod
    def _parse_line(line):
        if not line.startswith("CAM:"):
            return None

        payload = line[4:].strip()
        if not payload:
            return []

        blobs = []
        for part in payload.split(";"):
            fields = part.split(",")
            if len(fields) != 3:
                continue
            try:
                cx = int(fields[0])
                cy = int(fields[1])
            except ValueError:
                continue
            blobs.append([cx, cy, fields[2]])
        return blobs

    @staticmethod
    def update():
        uart = Camera._uart
        if uart is not None:
            try:
                available = uart.any()
                if available:
                    data = uart.read(available)
                    if data and state.clockwise != 0:
                        Camera._rx_buffer += data
                        if len(Camera._rx_buffer) > 512:
                            Camera._rx_buffer = Camera._rx_buffer[-512:]

                        while b"\n" in Camera._rx_buffer:
                            line_bytes, Camera._rx_buffer = Camera._rx_buffer.split(b"\n", 1)
                            line = line_bytes.decode("ascii", "ignore").strip()
                            blobs = Camera._parse_line(line)
                            if blobs is not None:
                                Camera._last_blobs = blobs
            except Exception as e:
                print("Camera UART read error:", e)

        blobs = Camera._last_blobs.copy()
 
        for cx, cy, color in blobs:
            get_colour_position(cx, cy, color)
        return blobs

    @staticmethod
    def get_blobs():
        return Camera.update()


def get_colour_position(traffic_x, traffic_y, signal):
    camera_width = 160
    camera_height = 120
    camera_center_x = camera_width / 2
    camera_center_y = camera_height

    max_angle_difference = 20


    dx = traffic_x - camera_center_x
    dy = traffic_y - camera_center_y

    distance = math.sqrt(dx**2 + dy**2)

    if distance > 65:
        return

    relative_angle = math.degrees(math.atan2(dy, dx))
    relative_angle = (relative_angle + 180) % 360 - 180

    possible_traffic_positions = get_traffic_lane_inside_pov()

    for position in possible_traffic_positions:
        #if abs(position["angle"] - relative_angle) <= max_angle_difference:
        Field.set_signal(position["pos"], position["side"], signal)
        return


def get_traffic_lane_inside_pov():
    traffic_coord = [
        # Bottom Lane
        [100, 60, 0, 0],
        [150, 60, 1, 0],
        [200, 60, 2, 0],
        [240, 100, 3, 0]
    ];

    if not state.is_lane_with_parking:
        traffic_coord.append([100, 40, 0, 1])
        traffic_coord.append([150, 40, 1, 1])
        traffic_coord.append([200, 40, 2, 1])

    elif state.relative_lane != 3:
        traffic_coord.append([260, 100, 3, 1])

    x, y = state.get_relative_odom
    x_camera = x + robot_distance_from_camera * math.sin(state.compass_angle_relative)
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
                
                position_inside_pov.append({
                    "x": lane[0],
                    "y": lane[1],
                    "pos": lane[2],
                    "side": lane[3],
                    "angle": relative_angle,
                    "distance": distance,
                })
    return position_inside_pov

