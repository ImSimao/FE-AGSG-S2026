import math
import time
from machine import freq

freq(int(2.5 * 100_000_000))

from camera import Camera, _camera_pose, robot_distance_from_camera
from compass import Compass
from encoder import Encoder
from state import state

# Camera.update() only reads UART when clockwise != 0
state.clockwise = 1
state.current_lane = 0

# --- track the robot from the encoder + IMU, so the field position of a FIXED
#     sign stays put while the robot moves.
h0 = Compass.heading()          # heading at the start == 0 deg (facing +x)
state.compass_angle = 0.0
x_true, y_true = 150.0, 60.0
state.odom_x = 303 - x_true     # lane 0, clockwise +1
state.odom_y = y_true
last_dist = Encoder.distance_cm()

Camera.initialize(uart_id=1, rx_pin=9)

print("Debug camera - obstacle coordinates (tracking encoder + IMU)")
print("Start pose: x=150 y=60 heading=0 (clockwise=1)")
print("")

last_key = None


def update_pose():
    """Integrate the encoder delta along the current heading (heading 0 = +x)."""
    global last_dist
    dist = Encoder.distance_cm()
    delta = dist - last_dist
    last_dist = dist

    h = Compass.heading() - h0
    state.compass_angle = h
    rad = math.radians(h)

    x = 303 - state.odom_x
    y = state.odom_y
    x += delta * math.cos(rad)
    y += delta * math.sin(rad)
    state.odom_x = 303 - x
    state.odom_y = y


while True:
    update_pose()
    blobs = Camera.get_blobs()

    key = tuple(tuple(b) for b in blobs)
    if key == last_key:
        time.sleep_ms(100)
        continue
    last_key = key

    if not blobs:
        print("No detections")
    else:
        x_cam, y_cam, angle = _camera_pose()
        sin_a = math.sin(angle)
        cos_a = math.cos(angle)
        print("--- %d sign(s) ---" % len(blobs))
        for x, y, h, color in blobs:
            # Robot frame: origin at the rear axle, +x forward, +y right (local).
            rx = y + robot_distance_from_camera
            ry = x
            # Absolute field position (same transform as match_frame).
            fx = x_cam + y * cos_a + x * sin_a
            fy = y_cam + y * sin_a - x * cos_a
            print("%-5s | cam=(%6.1f,%6.1f) h=%4.1f | robot=(%6.1f,%6.1f) | field=(%6.1f,%6.1f)"
                  % (color, x, y, h, rx, ry, fx, fy))

    time.sleep_ms(100)
