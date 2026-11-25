import time
from distance import Distance
from state import state
from servo import Servo
import math
from pid import PIDController
from telemetry import Telemetry
from math import sin, cos


ROTATE_TIMEOUT_MS = 8 * 1000
MOVE_TIMEOUT_MS = 60 * 1000

# Measured braking distance (cm) as a function of speed (cm/s):
#   s(v) = BRAKE_DIST_INTERCEPT + BRAKE_DIST_SLOPE * v
# It includes the ~50-100 ms cruise-control delay, so it is roughly linear
# in v (the delay term v*t_delay dominates at high speed).
BRAKE_DIST_INTERCEPT = 1.0
BRAKE_DIST_SLOPE = 0.08

STOP_SPEED = 1.0   # cm/s, below this the robot is considered stopped

# The speed the lateral steering gains were tuned at (move_coordinates' own
# max_speed). The PID below is fed a path-domain dt normalised to this, so the
# same steering authority holds at any speed - see the steering block.
MOVE_REF_SPEED = 50.0

# Heading correction for the slow bay search. The lateral PID only sees the
# cross-track offset, so a car that enters the move already crabbed drives
# parallel to the line but off it; at the 15 cm/s of detect_parking_wall the
# offset grows so slowly that the correction never arrives in time and the car
# reaches the bay still crooked. These two set how much straightening is mixed
# in: degrees of steering per degree of heading error, and its share of the
# servo range so the lateral PID keeps authority.
HEADING_GAIN = 0.8
HEADING_MAX_DEG = 20.0


def get_angle_to_rotate(dest_x, dest_y):
    x_initial, y_initial = state.get_relative_odom

    # Absolute coordinates
    A = (x_initial, y_initial)
    B = (dest_x, dest_y)

    compass_angle = state.compass_angle_relative

    if compass_angle > 180:
        compass_angle -= 360

    # Odom: dy = sin(heading)*clockwise — for clockwise=-1, forward in relative
    # frame matches atan2 bearing -heading, not +heading.
    if state.clockwise == -1:
        compass_angle = -compass_angle

    angle_to_coord = math.degrees(math.atan2(dest_y - y_initial, dest_x - x_initial))

    angle_to_rotate = angle_to_coord - compass_angle

    if angle_to_rotate > 180:
        angle_to_rotate -= 360

    if angle_to_rotate < -180:
        angle_to_rotate += 360

    return angle_to_rotate

def _trace_call(name, dest_x, dest_y, extra=""):
    x, y = state.get_relative_odom
    Telemetry.log("call %s to (%.0f,%.0f) from (%.0f,%.0f) h=%.0f%s t=%d"
                  % (name, dest_x, dest_y, x, y, state.compass_angle_relative, extra,
                     time.ticks_ms()))


def _trace_end(name, reason):
    """How a move/rotate ended: the caller's stop, the timeout, or the target."""
    x, y = state.get_relative_odom
    Telemetry.log("end %s %s at (%.0f,%.0f) h=%.0f v=%.0f t=%d"
                  % (name, reason, x, y, state.compass_angle_relative,
                     state.raw_speed, time.ticks_ms()))


def rotate_coordinates(dest_x, dest_y, reverse = False):
    """Turn until the car points at a coordinate. Do not travel to it.

    The bearing is taken once, from where the car stands when the call starts.
    The old loop re-read it every 10 ms from the live position, so while the car
    curved towards the point the bearing moved with it, the aligned test never
    fired, and the loop only ended when the car was *on* the point. That turned
    every "rotation" into an arc of up to the full distance: from the parking
    bay in ccw it was 60 cm of travel to (200,60) instead of a 46 degree turn,
    straight over the traffic sign standing in that slot.

    Covering distance belongs to move_coordinates(), which calls this to line up
    and then drives; where a manoeuvre needs the car to come in on a curve, that
    curve has to be written as travel, not borrowed from the turn.
    """
    _trace_call("rotate_coords", dest_x, dest_y, " reverse" if reverse else "")
    if target_in_rotation_area(dest_x, dest_y):
        reverse = not reverse

    odom_x, odom_y = state.get_relative_odom
    dx, dy = dest_x - odom_x, dest_y - odom_y

    # math.hypot() does not exist in MicroPython's math (the module is a subset
    # of CPython's): on the board this raised AttributeError before the car had
    # turned once. math.sqrt() is available everywhere.
    if math.sqrt(dx * dx + dy * dy) < 3:
        # Nothing to turn to: the car is already on the point.
        _trace_end("rotate_coords", "sem rumo")
        return

    bearing = math.degrees(math.atan2(dy, dx))
    # The compass runs mirrored in a ccw round: world heading 46 reads as -46
    # here, which is why get_angle_to_rotate negates it.
    heading = bearing if state.clockwise == 1 else -bearing
    rotate_angle(heading, reverse)
    _trace_end("rotate_coords", "aligned")


def rotate_angle(angle, reverse = False):
    _trace_call("rotate_angle", angle, 0, " reverse" if reverse else "")
    start_ms = time.ticks_ms()
    while True:
        if time.ticks_diff(time.ticks_ms(), start_ms) > ROTATE_TIMEOUT_MS:
            Servo.set_angle(0)
            state.target_speed = 0
            _trace_end("rotate", "timeout")
            break

        angle_to_rotate = angle - state.compass_angle_relative

        if angle_to_rotate > 180:
            angle_to_rotate -= 360

        if angle_to_rotate < -180:
            angle_to_rotate += 360

        servo_angle = Servo._MAX_STEERING_OFFSET

        if abs(angle_to_rotate) < 15:
            state.target_speed = 10
            servo_angle = 0.6 * Servo._MAX_STEERING_OFFSET
        elif abs(angle_to_rotate) < 25:
            state.target_speed = 12
        else:  
            state.target_speed = 17
        
        if abs(angle_to_rotate) < 2:
            Servo.set_angle(0)
            state.target_speed = 0
            break

        if reverse:
            servo_angle = -servo_angle
            state.target_speed = -state.target_speed

        if angle_to_rotate > 0:
            Servo.set_angle(servo_angle)
        else:
            Servo.set_angle(-servo_angle)

        time.sleep(1/100)
    time.sleep(0.075)

def move_coordinates(dest_x, dest_y, reverse = False, rotate = True, detect_parking_wall = False, stop_when = None):
    _trace_call("move", dest_x, dest_y, " reverse" if reverse else "")

    desaccelerate_distance = 30
    max_speed = 50   # below the motor limit (~50 cm/s at 500 RPM) to avoid saturation/slip
    min_speed = 12   # motor cannot creep below ~12 cm/s (static friction)

    if detect_parking_wall:
        max_speed = 15

    x_initial, y_initial = state.get_relative_odom

    if rotate:
        if reverse:
            rotate_coordinates(x_initial- (dest_x - x_initial), y_initial- (dest_y - y_initial))
        else:
            # Face the target only when the car is not already pointing at it.
            # Turning for a couple of degrees at every waypoint turned the lap
            # into a staircase of stop-turn-drive steps; the lateral PID below
            # already holds the line for the remaining small error.
            if abs(get_angle_to_rotate(dest_x, dest_y)) > 12:
                rotate_coordinates(dest_x, dest_y)

            


    x_initial, y_initial = state.get_relative_odom

    # Absolute coordinates
    A = (x_initial, y_initial)
    B = (dest_x, dest_y)

    # Vector AB
    ABx = B[0] - A[0]
    ABy = B[1] - A[1]

    # Length of AB
    L = math.sqrt(ABx*ABx + ABy*ABy)

    if L < 1e-6:
        _trace_end("move", "no-op (destino = posicao)")
        return

    # Unit vector in the AB direction (x' axis)
    ux = ABx / L
    uy = ABy / L

    # Perpendicular vector (y' axis)
    ux_perp = -uy
    uy_perp = ux

    pid_y = PIDController(kp=0.45, ki=0.003, kd=0.75)

    # Heading the AB line itself asks for, in the odom frame. Used as the
    # reference for the straightening term during the bay search: unlike a
    # bearing to the destination it does not change as the car closes in, so it
    # still means something at 20 cm from the target.
    target_heading = math.degrees(math.atan2(ABy, ABx))
    move_start_ms = time.ticks_ms()
    last_loop_ms = move_start_ms

    while True:
        now_ms = time.ticks_ms()
        dt = time.ticks_diff(now_ms, last_loop_ms) / 1000.0
        last_loop_ms = now_ms

        # Optional early stop: keep moving continuously until the caller's
        # condition becomes true (e.g. the camera sees the lane colour).
        if stop_when is not None and stop_when():
            state.target_speed = 0
            Servo.set_angle(0)
            pid_y.reset()
            _trace_end("move", "stop_when")
            break

        if time.ticks_diff(now_ms, move_start_ms) > MOVE_TIMEOUT_MS:
            state.target_speed = 0
            pid_y.reset()
            Servo.set_angle(0)
            _trace_end("move", "timeout")
            break

        C = state.get_relative_odom

        # Vector AC
        ACx = C[0] - A[0]
        ACy = C[1] - A[1]

        # Coordinates in the rotated frame
        x_prime = ACx*ux + ACy*uy
        y_prime = ACx*ux_perp + ACy*uy_perp
        remaining_distance = L - x_prime

        
        # Brake anticipation: the robot still travels s(v) cm after the stop
        # command, so brake that early to stop at the target. Hand control back
        # immediately: waiting here for the robot to physically stop added
        # 0.5-1 s to every routine transition.
        brake_distance = BRAKE_DIST_INTERCEPT + BRAKE_DIST_SLOPE * abs(state.raw_speed)
        if remaining_distance <= brake_distance or remaining_distance < 0:
            state.target_speed = 0
            Servo.set_angle(0)
            pid_y.reset()
            _trace_end("move", "chegou")
            break

        if remaining_distance < desaccelerate_distance:
            state.target_speed = remaining_distance / desaccelerate_distance * (max_speed - min_speed) + min_speed
        else:
            state.target_speed = max_speed

        # --- steering, in PATH space --------------------------------------
        # The lateral PID is kd*dY/dt, and dY/dt = v*dY/ds: the derivative term
        # is therefore kd*v times the car's heading error. At the 15 cm/s the
        # parking search runs at (detect_parking_wall) it is 3.3x weaker than at
        # the 50 cm/s these gains were tuned at, so a heading error is only
        # answered once the car has already drifted off the line - "at this
        # speed it has to turn more". Feeding the PID a path-domain dt
        # (dt * v / MOVE_REF_SPEED) makes all three terms a function of the
        # distance travelled instead of the time spent, which is what the
        # geometry needs; at v = MOVE_REF_SPEED the two dts are identical, so
        # the fast moves keep the tuning they had.
        v_now = abs(state.current_speed)
        dt_path = dt * v_now / MOVE_REF_SPEED
        if dt_path < 0.1 * dt:
            dt_path = 0.1 * dt          # never divide the derivative by ~0
        elif dt_path > 2.0 * dt:
            dt_path = 2.0 * dt

        servo_angle = pid_y.compute(-y_prime, dt_path)

        # Straightening, bay search only. The PID above answers an offset that
        # at 15 cm/s barely builds up; this answers the heading directly, so the
        # wheels turn hard the moment the car is crooked instead of waiting for
        # it to drift. Sign is the same frame as -y_prime: a positive error is
        # corrected by a positive servo_angle before the clockwise mirror below.
        if detect_parking_wall:
            compass_now = state.compass_angle_relative
            if compass_now > 180:
                compass_now -= 360
            if state.clockwise == -1:
                compass_now = -compass_now

            heading_error = target_heading - compass_now
            if heading_error > 180:
                heading_error -= 360
            elif heading_error < -180:
                heading_error += 360

            straighten = HEADING_GAIN * heading_error
            if straighten > HEADING_MAX_DEG:
                straighten = HEADING_MAX_DEG
            elif straighten < -HEADING_MAX_DEG:
                straighten = -HEADING_MAX_DEG

            servo_angle += straighten

        # Deadband: below ~1,5 cm of lateral offset the odometry noise (a few mm)
        # would otherwise drive the steering into a permanent micro-weave.
        #if abs(y_prime) < 0.15:
        #    servo_angle = 0

        if reverse:
            state.target_speed = -state.target_speed
            servo_angle = -servo_angle

        Servo.set_angle(servo_angle*state.clockwise)

        time.sleep(1/100)


def circle_center_left(xr, yr, theta, radius):
    xc = xr - radius * sin(theta)
    yc = yr + radius * cos(theta)
    return xc, yc

def circle_center_right(xr, yr, theta, radius):
    xc = xr + radius * sin(theta)
    yc = yr - radius * cos(theta)
    return xc, yc

def is_inside_circle(xt, yt, xc, yc, radius):
    dx = xt - xc
    dy = yt - yc
    return (dx * dx + dy * dy) <= (radius * radius)

def target_in_left_rotation_area(xr, yr, theta, xt, yt, radius):
    xc, yc = circle_center_left(xr, yr, theta, radius)
    return is_inside_circle(xt, yt, xc, yc, radius)

def target_in_right_rotation_area(xr, yr, theta, xt, yt, radius):
    xc, yc = circle_center_right(xr, yr, theta, radius)
    return is_inside_circle(xt, yt, xc, yc, radius)

def target_in_rotation_area(xt, yt, radius=35):
    xr, yr = state.get_relative_odom
    # Same mirror get_angle_to_rotate() applies on counter-clockwise runs. Without
    # it the two functions disagree about which way the car faces: this test
    # decides to reverse for a target that is actually in front (and the other way
    # round), so on ccw the arc starts in the wrong sense, overshoots and flips
    # back. Clockwise runs are untouched - the mirror only applies when
    # state.clockwise == -1.
    heading = -state.compass_angle_relative if state.clockwise == -1 else state.compass_angle_relative
    theta = math.radians(heading)

    left = target_in_left_rotation_area(xr, yr, theta, xt, yt, radius)
    right = target_in_right_rotation_area(xr, yr, theta, xt, yt, radius)
    return left or right