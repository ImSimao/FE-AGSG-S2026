import time
import math
from background import get_odom_side_sonar
from field import Field, Lane, is_color
from  move_coordinates import move_coordinates, rotate_angle, rotate_coordinates
from distance import Distance
from servo import Servo
from state import State, state
from telemetry import Telemetry
try:
    import os as _os
except ImportError:
    _os = None


def _env_flag(name):
    """Read a "1"/else test flag from the environment, board-safe.

    MicroPython's ``os`` is not CPython's: the ESP32 port has no process
    environment, so ``os.environ`` does not exist and reading it raised
    ``AttributeError: 'module' object has no attribute 'environ'`` on the car
    while the same file ran fine in the simulator. The flags only exist to
    iterate from the sim, so on the board they are simply off.
    """
    if _os is None:
        return False
    try:
        # Deliberate CPython-only probe: the except is what keeps the board
        # alive. tools/mpy_audit.py accepts the marker on the reported line.
        return _os.environ.get(name) == "1"  # mpy-ok
    except AttributeError:
        return False


#: Test hook: run only the parking routine (see the call site).
test_parking = _env_flag("WRO_TEST_PARKING")

#Coordinates




ninety_degrees_distance_offset = 16 # 16.5

traffic_lane_center = (state.outer_wall - state.inner_wall) / 4
traffic_lane_y_offset = 28 #26.25
traffic_lane_y_parking = 11.5
first_obstacle_camera_coord = (100, traffic_lane_center)
second_obstacle_camera_coord = (200, traffic_lane_center)
first_traffic_lane_camera_x = 152
parking_gap = 30


def get_traffic_lane_y(traffic_side, rotate = False, reverse = False):
        return traffic_lane_center \
        + ((traffic_lane_y_offset * -traffic_side) if not state.is_lane_with_parking or traffic_side == -1 \
            else - traffic_lane_y_parking ) \
             - (ninety_degrees_distance_offset * -traffic_side if rotate else 0)
 



def entry_colour_known():
    """Is the section's *entry* sign actually in the table?

    The shortcut that skips the mid-lane stop asks whether the two sides came out
    equal. When no colour has been read yet both sides are the [-1,-1] default,
    they compare equal, the shortcut fires, and the car drives the whole section
    on a line nobody chose: in the third corridor it kept the entry line past the
    exit sign (measured: one single colour wait in a whole round, for the
    corridor that had already read both of its signs).
    """
    return any(is_color(signal) for signal in Field.LANES[state.relative_lane].zones[0])


def exit_colour_known():
    """Is the section's exit sign actually in the table (not just "empty enough")?

    `can_exit_lane()` also returns True when both zones look *effectively empty*
    - unknown, and out of view for two seconds - which is the right test for
    "may I enter the next lane" and the wrong one for "do I know the exit
    colour". Treating that as knowledge kept the entry colour for the whole
    section: in the bottom lane the car covers the 100 cm of the section in
    about two seconds, the zones crossed the two-second threshold right at the
    middle, and the exit sign (still 50 cm away, so not yet in the camera's
    63 cm range) was never read. The car then passed it on the entry line.
    """
    return any(is_color(signal) for signal in Field.LANES[state.relative_lane].zones[2])


def log_lane_line(tag, sides=None):
    """One line per decision: which sides the field says, which y they mean.

    The car's line inside a section is picked from the colours registered in
    that lane's zones, and nothing else records *why* a section was driven wide.
    Two of these per section make the choice readable from the exported round.

    ``sides`` is the latched pair when the line was already chosen: a colour
    that arrives after the latch changes the live table but not the line, and
    logging the live table made the export claim an exit line the car never
    drove.
    """
    if sides is None:
        sides = Field.lanes_sides()
    lane = Field.LANES[state.relative_lane % 4]
    zones = "".join(
        {"RED": "R", "GREEN": "G"}.get(str(signal), ".") for zone in lane.zones for signal in zone
    )
    odom = state.get_relative_odom
    Telemetry.log(
        "line %s lane=%d sides=%s y_mid=%.1f y_exit=%.1f zones=%s odom=(%.1f,%.1f) h=%.0f"
        % (tag, state.relative_lane, sides, get_traffic_lane_y(sides[0]),
           get_traffic_lane_y(sides[1]), zones, odom[0], odom[1],
           state.compass_angle_relative)
    )


def _angdiff(a, b):
    """Signed a - b in degrees, wrapped to +/-180 (the compass sits near 360)."""
    d = a - b
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d


def _recenter_x(target_x, rtp, speed=8):
    """Drive straight (servo 0) to return the axle's physical x to target_x.

    Uses odometry. It is NOT reliable as an absolute reference - a run-to-run
    offset of 0.3-3 cm accumulates in the entry manoeuvre - but it IS reliable
    for *relative* motion: the offset stays constant through the shimmy, so
    driving back a drift measured in the same frame cancels it. The caller
    passes a target captured this run, not an absolute value.

    rtp is rotate_right_to_park: +1 the nose ends west (forward = -x), -1 the
    nose ends east (forward = +x).
    """
    Servo.set_angle(0)
    fwd_x = -rtp  # +1 nose-west -> forward -x; -1 nose-east -> forward +x
    for _ in range(60):  # ~3 s cap
        rel_x = state.get_relative_odom[0]
        phys_x = (State.outer_wall - rel_x) if state.clockwise == 1 else rel_x
        err = phys_x - target_x
        if abs(err) < 0.25:
            break
        need_neg_x = err > 0  # too far east -> move west (-x)
        go_forward = need_neg_x == (fwd_x == -1)
        state.target_speed = speed if go_forward else -speed
        time.sleep(0.05)
    state.target_speed = 0


def _bay_swing(degrees, lock, forward=True, speed=12, lead=0.0):
    """Swing the body by `degrees` with the wheels held at full lock.

    The bay is 1,5 x the chassis (190,5 mm) and the car is 127 mm long: a driven
    arc reaches the magenta element after ~5 cm, so the car cannot turn out of
    the bay in one movement. Two opposite-lock swings rotate the body ~33 deg
    inside the bay - measured on the compass, not on a distance, because the
    limit is the body corner and not the nose - and from there the exit run
    crosses x = 179 at y = 26, clear of the element.

    ``lead`` stops the swing that many degrees early to compensate the compass
    lag (~30 ms): without it a 3 deg swing coasted to 4-9 deg.
    """
    start = state.compass_angle_relative
    Telemetry.log("bay: swing %.0f deg, lock %+d, %s, from h=%.0f"
                  % (degrees, lock, "forward" if forward else "reverse", start))
    # Command the swing once, then poll the compass tightly and stop the instant
    # it reaches the target (minus the lead).
    Servo.set_angle(lock * Servo._MAX_STEERING_OFFSET)
    state.target_speed = speed if forward else -speed
    state.target_speed /= 2
    time.sleep(0.25)
    
    t0 = time.ticks_ms()
    state.target_speed = speed if forward else -speed
    target = max(0.0, degrees - lead)
    while (abs(_angdiff(state.compass_angle_relative, start)) < target
           and time.ticks_diff(time.ticks_ms(), t0) < 3000):
        time.sleep(1/200)
    state.target_speed = 0
    Servo.set_angle(0)
    Telemetry.log("bay: swing end h=%.0f (%+.0f), odom=%s"
                  % (state.compass_angle_relative,
                     _angdiff(state.compass_angle_relative, start), state.get_relative_odom))
    time.sleep(0.05)


def obstacle_challenge():
    global first_traffic_lane_camera_x
    time.sleep(0.15)

    # Wait for real echoes before choosing the direction. The firmware restarts
    # with the world already running (the UI's repeat button), and its first
    # sleep is short: at 4x a missing frame at this instant used to make
    # get_left() return -1, the comparison below flip the round to
    # counter-clockwise, and the car drive out of the bay into the wall.
    # All four, not just the sides: on a repeat the firmware restarts into a
    # world that is already running, and at 4x its first sleep is 250 ms of wall
    # clock - short enough that the rear reading can still be missing (-1) when
    # the boot geometry is built from it.
    # Wait for the sonar stream to flow before choosing the direction. Only
    # front/rear are guaranteed in range at the start pose: the side facing the
    # far wall is beyond the SRF04's ~250 cm range, so it reads -1 (no echo)
    # *permanently* - never "valid". Requiring all four made every clockwise
    # boot burn the full wait and then fall into the mirrored exit. The cap is
    # generous (3 s of sim time) because at 4x the first sonar frame can trail
    # the firmware's boot by more than the old 1 s window.
    for _ in range(300):
        if Distance.get_front() >= 0 and Distance.get_rear() >= 0:
            break
        time.sleep(0.01)

    left = Distance.get_left()
    right = Distance.get_right()
    # -1 is "no echo": the wall is beyond range, i.e. FAR, not close. The bay
    # sits against the outer wall, so on a clockwise start the far wall is on the
    # right (left=16 cm, right=287 cm->-1) and on a ccw start it is on the left.
    # Comparing -1 as if it were a real centimetre made "16 < -1" False and
    # defaulted the clockwise round to the mirrored ccw exit.
    if left < 0:
        left = float("inf")
    if right < 0:
        right = float("inf")
    state.clockwise = 1 if left < right else -1

    Telemetry.log("boot sonar L=%.1f R=%.1f F=%.1f rear=%.1f cw=%d"
                  % (Distance.get_left(), Distance.get_right(), Distance.get_front(),
                     Distance.get_rear(), state.clockwise))

    distance_left, distance_right, distance_front, distance_rear = get_odom_side_sonar()

    if test_parking:
        # Test hook: skip the exit and the three laps, run only the parking
        # routine, for fast iteration on the shimmy. The car must start at the
        # "end of lap 3" pose: ?pose=216,43,180 (physical x=216, y=43, facing
        # west). The odom mirrors x for cw: 303 - 216 = 87.
        state.set_relative_odom(87 if state.clockwise == 1 else 216, 43)
        Telemetry.log("TEST_PARKING: skipping to parking routine (cw=%d)" % state.clockwise)
    else:
        searched_traffic_sign = False

        if state.clockwise == 1:
            state.set_relative_odom(100+1.75, distance_left)

            #for _ in range(4):
            #    _bay_swing(7, state.clockwise, forward=True)
            #    _bay_swing(3, -state.clockwise, forward=False)

            rotate_coordinates(150, 60)

            time.sleep(0.1)

            if not Field.can_exit_lane():
                move_coordinates(150, 30, rotate=False)
                rotate_coordinates(200, 60)

                searched_traffic_sign = True

        else:
            state.set_relative_odom(200-distance_front, distance_right)

            #for _ in range(3):
            #    _bay_swing(7, state.clockwise, forward=True)
            #    _bay_swing(3, -state.clockwise, forward=False)

            rotate_angle(-60)

            t0 = time.ticks_ms()
            while not Field.can_exit_lane() and time.ticks_diff(time.ticks_ms(), t0) < 3000:
                time.sleep(0.1)
            #rotate_coordinates(200, 60)

        if Field.lanes_sides()[1] == -1:
            rotate_angle(90*state.clockwise, reverse=True if state.clockwise == -1 else False)
            move_coordinates(state.get_relative_odom[0], get_traffic_lane_y(Field.lanes_sides()[1], rotate=True))
        elif not searched_traffic_sign:
            rotate_angle(65*state.clockwise)
    
        rotate_angle(0, reverse= Field.lanes_sides()[1] == 1 and searched_traffic_sign)

        if state.get_relative_odom[0] < 200:
            move_coordinates(200, get_traffic_lane_y(Field.lanes_sides()[1]), rotate = False)
    
        Field.exiting_park = False
    
        laps = 3

        while state.lap <= laps:
            state.current_lane += 1

            # The lane's last zone is read again from scratch on every pass; the
            # colour left there at the end of the previous lap came from the next
            # lane's first sign and picked this section's exit line before the car
            # had seen its own third sign.
            Field.prepare_lane()

    
            if not Field.can_enter_lane():
                #search for colour
                Telemetry.log("search: section with no colour in zone 0/1 odom=(%.0f,%.0f) zones=%s t=%d"
                              % (state.get_relative_odom[0], state.get_relative_odom[1],
                                 "".join({"RED": "R", "GREEN": "G"}.get(str(s),
                                          ".") for z in Field.LANES[state.relative_lane].zones
                                         for s in z), time.ticks_ms()))
                if state.is_lane_with_parking:
                    move_coordinates(50, 45)
                else:
                    move_coordinates(50, 35)
            
                rotate_angle(0, reverse=True)
                
                rotate_coordinates(first_obstacle_camera_coord[0], first_obstacle_camera_coord[1] if not state.is_lane_with_parking else 60)
            
                # Search forward in one continuous move and stop as soon as the
                # camera detects the lane opening (no more 10 cm hops with a stop
                # between each one).
                if state.get_relative_odom[0] <= 100:
                    # can_enter_lane() is not "I have the colour": it is also true
                    # when the four entry slots are effectively empty, which is the
                    # state the search exists to leave. Stopping on it ended the move
                    # after 0 cm (measured: lane 1 and lane 3 did not move at all,
                    # lane 0 moved 2.1 cm) and the section then picked its line from
                    # the [-1,-1] default.
                    move_coordinates(110, 50 if not state.is_lane_with_parking else 60,
                                     rotate=False, stop_when=entry_colour_known)
            
                rotate_angle(-90*Field.lanes_sides()[0]*state.clockwise, True)
                move_coordinates(state.get_relative_odom[0], get_traffic_lane_y(Field.lanes_sides()[0], True), rotate=False)

            elif Field.lanes_sides()[0] == 1:
                move_coordinates(state.get_relative_odom[0], get_traffic_lane_y(Field.lanes_sides()[0], True), rotate=False)

            elif Field.lanes_sides()[0] == -1:
                if state.get_relative_odom[0] > 85:
                    move_coordinates(state.get_relative_odom[0], 65, rotate=False)
                    rotate_angle(0, reverse=True)
                elif state.get_relative_odom[1] > 90:
                    move_coordinates(state.get_relative_odom[0], state.get_relative_odom[1] - 5 , rotate=False)
    
    
            rotate_angle(0)
        
            if state.lap == laps:
                break

            # Latch the sides for this section. The lane's own signs are read on the
            # way in; anything that arrives later is the first sign of the *next*
            # lane, and letting it flip the choice half way down the section sent
            # the car from the red line (y=80) to the green one (y=20) between the
            # middle of the lane and its exit - 64 cm across the lane, past the red.
            section_sides = Field.lanes_sides()

            # The two sides only say something when both were actually read. With no
            # colour in the table they default to [-1,-1], compare equal, and the
            # shortcut below would skip the mid-lane stop for a section whose line
            # was never chosen.
            line_is_known = entry_colour_known() and exit_colour_known()

            if not (Field.can_exit_lane() and line_is_known and section_sides[0] == section_sides[1]) or state.lap == laps:
                log_lane_line("mid")
                move_coordinates(first_traffic_lane_camera_x, get_traffic_lane_y(section_sides[0]), rotate = False)


                if not (Field.can_exit_lane() and exit_colour_known()):
                    # Point at the lane centre, where the section's third sign sits,
                    # and not down the entry line: from the entry line the sign is
                    # 40 cm to the side, i.e. 49 deg off the camera axis at 57 cm
                    # (measured: odom=(149,83), sign at (200,40)). Outside the 70.8
                    # deg FOV it is not there at all, so the wait below always ran
                    # its full 3 s with an empty table and the car then drove on
                    # with the entry colour - it stood still looking at nothing.
                    # The edge case this replaces: aiming across the lane while
                    # still moving arcs the car over the line, so the turn happens
                    # here first, at the reduced speed rotate_coordinates uses.
                    rotate_coordinates(second_obstacle_camera_coord[0],
                                       second_obstacle_camera_coord[1])
                
                    start_time = time.ticks_ms()
                    Telemetry.log("waiting for exit colour odom=(%.0f,%.0f) zones=%s t=%d"
                                  % (state.get_relative_odom[0], state.get_relative_odom[1],
                                     "".join({"RED": "R", "GREEN": "G"}.get(str(s), ".")
                                             for z in Field.LANES[state.relative_lane].zones
                                             for s in z), time.ticks_ms()))
                    waited = 0
                    while not exit_colour_known():
                        time.sleep(0.1)
                        waited += 100

                        if time.ticks_ms() - start_time > 3000:
                            break
                    Telemetry.log("wait ended after %d ms, zones=%s t=%d"
                                  % (waited,
                                     "".join({"RED": "R", "GREEN": "G"}.get(str(s), ".")
                                             for z in Field.LANES[state.relative_lane].zones
                                             for s in z), time.ticks_ms()))
                    # The wait is the only moment the section's exit colour can
                    # arrive: the line was latched before the car could see that
                    # sign. When it arrived, take it. When the timeout fired, keep
                    # the entry colour - whatever is in the table then came from
                    # the next lane's corner sign, not from this section.
                    if exit_colour_known():
                        section_sides[1] = Field.lanes_sides()[1]

                    if section_sides[0] == section_sides[1]:
                        rotate_angle(0, reverse=True)

                # Take the exit side from the live table whether or not the wait
                # ran. The wait is skipped exactly when the exit colour is already
                # known - and that is the case where the latched side is at its most
                # misleading, because it was copied from the entry colour before
                # this lane read any of its own signs.
                if exit_colour_known():
                    section_sides[1] = Field.lanes_sides()[1]

                if section_sides[1] == -1 and state.get_relative_odom[1] < 50 or section_sides[1] == 1 and state.get_relative_odom[1] > 50:
                    rotate_angle(-90*section_sides[1]*state.clockwise)
                    
                    target = get_traffic_lane_y(section_sides[1], rotate=True)
                    diff = state.get_relative_odom[1] - target if section_sides[1] == 1 else target - state.get_relative_odom[1]
                    
                    if not state.is_lane_with_parking or diff > 3:
                        move_coordinates(state.get_relative_odom[0], get_traffic_lane_y(section_sides[1], rotate=True))
                    rotate_angle(0)

            # Store the line this lane produced: laps two and three drive it without
            # stopping to look again.
            Field.remember_lane(section_sides)
            log_lane_line("exit", section_sides)
            move_coordinates((State.inner_wall/2) + (State.outer_wall/2), get_traffic_lane_y(section_sides[1]), rotate = False)


    #parking
    if not (state.clockwise == 1 and state.get_relative_odom[1] < 60):      
        move_coordinates(100 if state.clockwise == 1 else 150, state.get_relative_odom[1], rotate = False)
    
    
    if state.get_relative_odom[1] > 60:
        if state.clockwise == 1:
            rotate_angle(-90 * state.clockwise)
            move_coordinates(state.get_relative_odom[0], 25, rotate=False)
            rotate_angle(-90 * state.clockwise)
            rotate_angle(0, reverse=True)
        else:
            rotate_angle(-90 * state.clockwise)
            move_coordinates(state.get_relative_odom[0], 50, rotate=False)
            rotate_angle(-45 * state.clockwise)
            rotate_angle(0, reverse=True)

            while state.get_relative_odom[0] > 200 - (parking_gap + 15):
                state.target_speed = -10
                time.sleep(0.1)

    rotate_angle(0)

    parking_offset = 5
    parking_rear_distance = 7
    rotation_margin_cm = 16


    # Move until detect first parking wall with side sonar
    move_coordinates((100) if state.clockwise == 1 else (200 - parking_gap), state.get_relative_odom[1], rotate=False, 
                         stop_when=lambda: 
                            abs(state.get_relative_odom[1] - Distance.get_left() ) > 20 if state.clockwise == 1 else abs(state.get_relative_odom[1] - Distance.get_right() ) > 20)


    state.set_relative_odom(100 - 8 if state.clockwise == 1 else 200 - parking_gap - 8, state.get_relative_odom[1])

    move_coordinates(100 + parking_offset if state.clockwise == 1 else 200 - (parking_gap - parking_offset), state.get_relative_odom[1], rotate=False)
 
    rotate_angle(90 * state.clockwise)

    time.sleep(0.5)

    rotate_angle(90 * state.clockwise, reverse=True)

    time.sleep(1)

    while True:
        diff_distance = Distance.get_rear() - parking_rear_distance

        if abs(diff_distance) < 2:
            state.target_speed = 0

            if Distance.get_left() < rotation_margin_cm and Distance.get_right() < rotation_margin_cm:
                if Distance.get_left() < Distance.get_right():
                    rotate_angle(90 * state.clockwise - 20)
                else:
                    rotate_angle(90 * state.clockwise + 20)
                rotate_angle(90 * state.clockwise)
                time.sleep(0.1)
            else:
                break
        elif diff_distance > 0:
            state.target_speed = -10
        else:
            state.target_speed = 10

        time.sleep(0.1)

    time.sleep(0.25)


    # The rotation direction must not trust the raw side sonars: they sit
    # 7.97 cm forward of the rear axle and shoot over the 20 cm-tall magenta
    # wall whenever the axle is above y ~12 cm, reading the far field (100+ cm)
    # instead of the near wall. A single bad reading flips "left < right" and
    # the nose swings into the wall. The odometry is reliable (a few cm), so
    # decide from where the car actually sits relative to the bay centre.
    rel_x = state.get_relative_odom[0]
    physical_x = (State.outer_wall - rel_x) if state.clockwise == 1 else rel_x
    # Bay centre is at 183 cm (physical): rightX 198 - gap/2 15. Right of
    # centre there is more room on the left, so rotate left (+1) and let the
    # nose swing into the open side.
    rotate_right_to_park = 1 if physical_x > 183 else -1
    Telemetry.log("bay: decide odom_x=%.1f phys=%.1f -> rotate_right_to_park=%+d" %
                  (rel_x, physical_x, rotate_right_to_park))


    while True:
        diff_distance = Distance.get_rear() - 14.5

        if abs(diff_distance) < 1:
            state.target_speed = 0
            break
        elif diff_distance > 0:
            state.target_speed = -10
        else:
            state.target_speed = 10

        time.sleep(0.1)

    time.sleep(0.25)

    # The shimmy drifts the car laterally. The side sonars cannot measure this
    # while the car is off-parallel (diagonal readings, they shoot over the
    # 20 cm wall), and the odometry's *absolute* x has a run-to-run offset of
    # 0.3-3 cm from the entry manoeuvre. But the odometry's *relative* motion is
    # faithful, so capture x here - the car is already at the correct parallel x
    # (the reverse into the bay leaves the chassis at ~188-190, close to where
    # it must finish) - and recentre back to it, cancelling the drift in the
    # same frame it is measured. No absolute offset is added: the entry already
    # sits at the finishing x.
    x_ref = (State.outer_wall - state.get_relative_odom[0]) if state.clockwise == 1 else state.get_relative_odom[0]
    Telemetry.log("bay: recenter x_ref=%.1f" % x_ref)

    while True:
        _bay_swing(3, -rotate_right_to_park, forward=True)
        _bay_swing(3, rotate_right_to_park, forward=False)
        

        compass_angle = state.compass_angle_relative
        if compass_angle > 180:
            compass_angle -= 360

        diff = min(abs(compass_angle), abs(180 - compass_angle))

        if diff < 45:
            _recenter_x(x_ref, rotate_right_to_park)

        if diff < 15:
            target = 0 if abs(compass_angle) < 90 else 180

            diff = target - compass_angle
            #if abs(diff) >= 8:
            #    _bay_swing(abs(diff/2), -1 if diff > 0 else 1, forward=False)

            state.target_speed = 0
            break