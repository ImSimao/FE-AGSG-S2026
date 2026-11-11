import math
import time
from machine import freq

freq(int(2.5 * 100_000_000))

from camera import Camera, _camera_pose, robot_distance_from_camera
from state import state

# Camera.update() only reads UART when clockwise != 0
state.clockwise = 1
state.current_lane = 0
state.compass_angle = 0.0
state.set_relative_odom(150, 60)

Camera.initialize(uart_id=1, rx_pin=9)

print("Debug camera - obstacle coordinates")
print("Robot pose: x=150 y=60 heading=0 (clockwise=1)")
print("")

last_key = None

while True:
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
            # The camera sits robot_distance_from_camera cm ahead of that origin.
            rx = y + robot_distance_from_camera  # forward -> robot x
            ry = x  # lateral (+right) -> robot y
            # Absolute field position (same transform as match_frame).
            fx = x_cam + y * cos_a + x * sin_a
            fy = y_cam + y * sin_a - x * cos_a
            print("%-5s | cam=(%6.1f,%6.1f) h=%4.1f | robot=(%6.1f,%6.1f) | field=(%6.1f,%6.1f)"
                  % (color, x, y, h, rx, ry, fx, fy))

    time.sleep_ms(100)
