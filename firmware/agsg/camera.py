import math
import time
from field import Field
from machine import UART, Pin
from state import state
from telemetry import Telemetry

camera_angle = 70
camera_distance = 75
robot_distance_from_camera = 13.5

# A blob is placed by bearing, not by being first in the list. The list is
# ordered by the field layout, so "first" meant "the leftmost sign of the lane"
# whatever the blob was: two signs in one frame both landed on it and the later
# one won, which is how a green from the next lane overwrote the red at the end
# of this one and the car drove the green line past a red.
CAMERA_WIDTH = 160
CAMERA_FOCAL_PX = 114.0
MAX_SIGN_ANGLE_DEG = 20.0
# Past this a 5 cm sign is a few pixels and the colour is a guess.
MAX_SIGN_DISTANCE_CM = 80.0


def blob_bearing_deg(traffic_x):
    return math.degrees(math.atan2(traffic_x - (CAMERA_WIDTH / 2 - 1), CAMERA_FOCAL_PX))


def match_frame(blobs, possible_traffic_positions):
    """Pair every blob of one frame with at most one lane position."""
    candidates = []
    for index, blob in enumerate(blobs):
        bearing = blob_bearing_deg(blob[0])
        for position in possible_traffic_positions:
            if position.get("distance", 0) > MAX_SIGN_DISTANCE_CM:
                continue
            error = abs(position["angle"] - bearing)
            if error <= MAX_SIGN_ANGLE_DEG:
                candidates.append((error, index, position, bearing))

    candidates.sort(key=lambda item: item[0])

    used_blobs = set()
    used_positions = set()
    for error, index, position, bearing in candidates:
        key = (position["pos"], position["side"])
        if index in used_blobs or key in used_positions:
            continue
        used_blobs.add(index)
        used_positions.add(key)
        Telemetry.log("blob %s -> pos %d err %.1f (%.0f cm)"
                      % (blobs[index][2], position["pos"], error, position.get("distance", 0)))
        Field.set_signal(position["pos"], position["side"], blobs[index][2])


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
    _last_pov_ms = None
    _last_pov_positions = []

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
        new_frame = False
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
                                new_frame = True
            except Exception as e:
                print("Camera UART read error:", e)

        if new_frame:
            Camera._last_pov_positions = get_traffic_lane_inside_pov()
            match_frame(Camera._last_blobs, Camera._last_pov_positions)

        
 

        Camera._accumulate_pov_time(Camera._last_pov_positions)
        return Camera._last_blobs.copy()

    @staticmethod
    def _accumulate_pov_time(possible_traffic_positions):
        now = time.ticks_ms()

        if state.clockwise == 0:
            Camera._last_pov_ms = now
            return

        if Camera._last_pov_ms is None:
            dt = 0
        else:
            dt = time.ticks_diff(now, Camera._last_pov_ms)
        Camera._last_pov_ms = now

        if dt <= 0:
            return

        for position in possible_traffic_positions:
            Field.add_pov_time(position["pos"], position["side"], dt)

    @staticmethod
    def get_blobs():
        return Camera.update()


def get_colour_position(traffic_x, traffic_y, signal, possible_traffic_positions):
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

    match_frame([(traffic_x, traffic_y, signal)], possible_traffic_positions)


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
    # The compass mirrors in a ccw round (see get_angle_to_rotate): with the raw
    # value every candidate landed outside the 35 degree half-FOV, match_frame
    # paired nothing, and the car drove the whole lane without registering a
    # single colour - the round ended on the first sign, on the wrong side.
    heading = state.compass_angle_relative
    if state.clockwise == -1:
        heading = -heading
    angle = math.radians(heading)
    x_camera = x + robot_distance_from_camera * math.sin(angle)
    y_camera = y + robot_distance_from_camera * math.cos(angle)

    position_inside_pov = []
    
    for lane in traffic_coord:
        dx = lane[0] - x_camera
        dy = lane[1] - y_camera
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance <= camera_distance:
           
            relative_angle = math.degrees(math.atan2(dy, dx)) - heading
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

