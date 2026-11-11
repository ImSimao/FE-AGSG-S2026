import math
import time
from field import Field
from machine import UART, Pin
from state import state
from telemetry import Telemetry

camera_angle = 70
camera_distance = 75
robot_distance_from_camera = 10.5

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

# A detected sign is matched to a field position when the two agree
# within this radius (in field cm).
MATCH_RADIUS_CM = 5.0


def blob_bearing_deg(traffic_x):
    return math.degrees(math.atan2(traffic_x - (CAMERA_WIDTH / 2 - 1), CAMERA_FOCAL_PX))


def apply_surprise_rule(color, height_cm):
    """Surprise Rule: a modified sign is 5 or 15 cm tall (instead of 10 cm) and
    carries the OPPOSITE colour of the original. The vehicle must pass on the
    side of the original colour, so a 5/15 cm sign's colour is inverted here.
    h == 0 means the height was unknown (a clipped sign), so no inversion.
    """
    if 3.5 <= height_cm < 7.5 or 12.5 < height_cm <= 18.0:
        return "GREEN" if color == "RED" else "RED"
    return color


def _camera_pose():
    """Absolute (x, y) of the camera and the robot heading, in field cm."""
    x, y = state.get_relative_odom
    heading = state.compass_angle_relative
    if state.clockwise == -1:
        heading = -heading
    angle = math.radians(heading)
    x_camera = x + robot_distance_from_camera * math.sin(angle)
    y_camera = y + robot_distance_from_camera * math.cos(angle)
    return x_camera, y_camera, angle


def match_frame(blobs, possible_traffic_positions):
    """Pair every blob of one frame with at most one field position.

    A blob is (x, y, h, COLOR) in centimetres from the camera: x is the lateral
    offset (+ right), y the forward distance, h the sign height. The sign's
    absolute field position is computed and matched to the nearest field
    position within MATCH_RADIUS_CM.
    """
    x_camera, y_camera, angle = _camera_pose()
    sin_a = math.sin(angle)
    cos_a = math.cos(angle)

    used_positions = set()
    for index, blob in enumerate(blobs):
        x, y, h, color = blob
        # sign position relative to the camera -> absolute field position
        sign_x = x_camera + y * sin_a + x * cos_a
        sign_y = y_camera + y * cos_a - x * sin_a

        best = None
        best_d = None
        for position in possible_traffic_positions:
            key = (position["pos"], position["side"])
            if key in used_positions:
                continue
            dx = sign_x - position["x"]
            dy = sign_y - position["y"]
            d = math.sqrt(dx * dx + dy * dy)  # math.hypot is missing in MicroPython
            if d <= MATCH_RADIUS_CM and (best_d is None or d < best_d):
                best = position
                best_d = d

        if best is not None:
            used_positions.add((best["pos"], best["side"]))
            effective = apply_surprise_rule(color, h)
            Telemetry.log("blob %s (h=%.1f) -> pos %d err %.1f cm"
                          % (color, h, best["pos"], best_d))
            Field.set_signal(best["pos"], best["side"], effective)


class Camera:
    """
    Static helper for reading traffic-light detections from the OpenMV camera.

    Expects newline-terminated lines from the camera UART in the form:

        CAM:x,y,h,COLOR;x,y,h,COLOR

    x is the lateral offset of the sign centre in cm (+ right), y the forward
    distance in cm, h the sign height in cm (5/10/15; 0 if clipped) and COLOR
    the colour actually seen. Empty frames are sent as ``CAM:``.
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
            if len(fields) != 4:
                continue
            try:
                x = float(fields[0])   # lateral offset of the sign centre (cm, + right)
                y = float(fields[1])   # forward distance of the sign centre (cm)
                h = float(fields[2])   # sign height (5 / 10 / 15 cm; 0 if clipped)
            except ValueError:
                continue
            blobs.append([x, y, h, fields[3]])   # [x, y, h, COLOR]
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

    match_frame([(traffic_x, traffic_y, 10.0, signal)], possible_traffic_positions)


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

    x_camera, y_camera, angle = _camera_pose()
    # The compass mirrors in a ccw round (see get_angle_to_rotate): with the raw
    # value every candidate landed outside the 35 degree half-FOV, match_frame
    # paired nothing, and the car drove the whole lane without registering a
    # single colour - the round ended on the first sign, on the wrong side.
    heading = math.degrees(angle)

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

