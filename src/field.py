from state import state
from telemetry import Telemetry


def is_color(traffic_signal):
    return traffic_signal == TrafficSignal.GREEN or traffic_signal == TrafficSignal.RED

def is_empty(traffic_signal):
    return traffic_signal == TrafficSignal.EMPTY

def is_unknown(traffic_signal):
    return traffic_signal == TrafficSignal.UNKNOWN

def is_green(traffic_signal):
    return traffic_signal == TrafficSignal.GREEN

ZONE_POV_THRESHOLD_MS = 2000


class TrafficSignal:
    GREEN = "GREEN"
    RED = "RED"
    UNKNOWN = "UNKNOWN"
    EMPTY = "EMPTY"


class Lane:
    """Four lanes around the field; each lane has three zones."""

    def __init__(self, lane_index):
        self.lane_index = lane_index
        self.zones = [
            [TrafficSignal.UNKNOWN, TrafficSignal.UNKNOWN],
            [TrafficSignal.UNKNOWN, TrafficSignal.UNKNOWN],
            [TrafficSignal.UNKNOWN, TrafficSignal.UNKNOWN],
        ]

        self.zone_pov_ms = [
            [0, 0],
            [0, 0],
            [0, 0],
        ]

        if lane_index == 0:
            for zone in self.zones:
                zone[1] = TrafficSignal.EMPTY



    def add_pov_time(self, zone_index, position_index, dt_ms):
        if zone_index == 3:
            Field.LANES[(self.lane_index + 1) % 4].add_pov_time(0, position_index, dt_ms)
            return

        self.zone_pov_ms[zone_index][position_index] += dt_ms

    def _is_effectively_empty(self, zone_index, position_index):
        signal = self.zones[zone_index][position_index]
        if is_empty(signal):
            return True
        if is_unknown(signal) and self.zone_pov_ms[zone_index][position_index] >= ZONE_POV_THRESHOLD_MS:
            return True
        return False

    def can_enter_zone(self, zone_indices):
        color_total = 0
        empty_total = 0

        for zone_index in zone_indices:
            for position_index in range(2):
                if is_color(self.zones[zone_index][position_index]):
                    color_total += 1
                elif self._is_effectively_empty(zone_index, position_index):
                    empty_total += 1

        return color_total != 0 or empty_total == 4

    def can_enter_lane(self):
        return self.can_enter_zone([0, 1])

    def can_exit_lane(self):
        return self.can_enter_zone([1, 2])

    def can_register_signal_in_zone(self, zones):
        for zone in zones:
            for traffic_signal in zone:
                if is_color(traffic_signal):
                    return False
        return True


    def set_signal(self, zone_index, position_index, signal):
        if zone_index == 3:
            Field.LANES[(self.lane_index + 1) % 4].set_signal(0, position_index, signal)
            return

        if zone_index == 0:
            if not self.can_register_signal_in_zone(self.zones[0:2]):
                return

        if zone_index == 1:
            if not self.can_register_signal_in_zone(self.zones):
                return

        if zone_index == 2:
            if not self.can_register_signal_in_zone(self.zones[1:3]):
                return

        
        Telemetry.log(f"Set signal in zone {zone_index} {position_index} {signal}")

        self.zones[zone_index][position_index] = signal


    def lanes_sides(self):
        #1 left side (green), -1 right side (red)
        zone_colors = [None, None, None]
        sides = [None, None]

        for k, zone in enumerate(self.zones):
            for traffic_signal in zone:
                if is_color(traffic_signal):
                    zone_colors[k] = traffic_signal

        if zone_colors[1] is not None:
            side = 1 if is_green(zone_colors[1]) else -1
            sides[0] = side
            sides[1] = side
        else:
            if zone_colors[0] is not None:
                sides[0] = 1 if is_green(zone_colors[0]) else -1

            if zone_colors[2] is not None:
                sides[1] = 1 if is_green(zone_colors[2]) else -1

        if sides[0] is None:
            sides[0] = sides[1]

        if sides[1] is None:
            sides[1] = sides[0]

        if state.clockwise == -1:
            if sides[0] == -1:
                sides[0] = 1
            else:
                sides[0] = -1
            
            if sides[1] == -1:
                sides[1] = 1
            else:
                sides[1] = -1

        if sides[0] is None or sides[1] is None:
            sides = [-1, -1]

        return sides

        
        

class Field:
    LANES = [
        Lane(0),
        Lane(1),
        Lane(2),
        Lane(3),
    ]

    exiting_park = True

    @staticmethod
    def set_signal(zone_index, position_index, signal):
        if Field.exiting_park and zone_index == 1:
            Field.exiting_park = False
            zone_index = 2
    
        Field.LANES[state.relative_lane].set_signal(zone_index, position_index, signal)

    @staticmethod
    def add_pov_time(zone_index, position_index, dt_ms):
        if Field.exiting_park and zone_index == 1:
            zone_index = 2

        Field.LANES[state.relative_lane].add_pov_time(zone_index, position_index, dt_ms)

    @staticmethod
    def can_enter_lane():
        return Field.LANES[state.relative_lane].can_enter_lane()
    
    @staticmethod
    def can_exit_lane():
        return Field.LANES[state.relative_lane].can_exit_lane()

    @staticmethod
    def lanes_sides():
        return Field.LANES[state.relative_lane].lanes_sides()

    @staticmethod
    def parking_traffic_exit_confirmation():
        color_total = 0
        color_unknown = 0

        zones = Field.LANES[state.relative_lane].zones
        for zone in zones[1:3] if state.clockwise == 1 else zones[2:3]:
            if is_color(zone[0]):
                color_total += 1
            if is_unknown(zone[0]):
                color_unknown += 1
        
        return color_total == 1 or color_unknown == 0