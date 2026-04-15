class State:
    """Static-style container for robot self."""

    def __init__(self):
        self.clockwise = 0
        self.current_lane = 0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.initial_compass_angle = 0.0
        self.compass_angle = 0.0
        self.current_speed = 0.0
        self.target_speed = 0.0

    @property
    def relative_lane(self):
        return self.current_lane % 4

    @property
    def is_lane_with_parking(self):
        return self.relative_lane == 0

    @property
    def lap(self):
        return self.current_lane // 4

    @property
    def compass_angle_relative(self):
        return (self.compass_angle + 
        (self.relative_lane * -90.0 * self.clockwise) + 360.0) % 360.0

    def set_relative_odom (self, x_relative, y_relative):
        if self.relative_lane == 0:
            if self.clockwise == -1:
                self.odom_x = x_relative
                self.odom_y = y_relative
            else:
                self.odom_x = 300-x_relative
                self.odom_y = y_relative
        elif self.relative_lane == 1:
            if self.clockwise == -1:
                self.odom_x = 300-y_relative
                self.odom_y = x_relative
            else:
                self.odom_x = y_relative
                self.odom_y = x_relative
        elif self.relative_lane == 2:
            if self.clockwise == -1:
                self.odom_x = 300-x_relative
                self.odom_y = 300-y_relative
            else:
                self.odom_x = x_relative
                self.odom_y = 300-y_relative
        elif self.relative_lane == 3:
            if self.clockwise == -1:
                self.odom_x = y_relative
                self.odom_y = 300-x_relative
            else:
                self.odom_x = 300-y_relative
                self.odom_y = 300-x_relative

    @property
    def get_relative_odom(self):
        if self.relative_lane == 0:
            if self.clockwise == -1:
                return self.odom_x, self.odom_y
            else:
                return 300-self.odom_x, self.odom_y
        elif self.relative_lane == 1:
            if self.clockwise == -1:
                return self.odom_y, 300-self.odom_x
            else:
                return self.odom_y, self.odom_x
        elif self.relative_lane == 2:
            if self.clockwise == -1:
                return 300-self.odom_x, 300-self.odom_y
            else:
                return self.odom_x, 300-self.odom_y
        elif self.relative_lane == 3:
            if self.clockwise == -1:
                return 300-self.odom_y, self.odom_x
            else:
                return 300-self.odom_y, 300-self.odom_x

state = State()