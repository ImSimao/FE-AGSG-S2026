import math
from state import state


# Encoder scale error (fraction of distance) plus a small count-noise floor.
_ALONG_SCALE = 0.02
_ALONG_FLOOR_CM = 0.05
# Heading is not in the state. A 2 degree compass error becomes lateral
# position error in proportion to the distance travelled this step.
_HEADING_SIGMA_RAD = math.radians(2.0)

# Sonar position fix, once the geometric gates have accepted the reading.
_SONAR_SIGMA_CM = 2.0
_SONAR_VAR = _SONAR_SIGMA_CM * _SONAR_SIGMA_CM

# Squared Mahalanobis gate. 9 is (3 sigma)^2 for a single axis.
_GATE = 9.0

# Covariance written by a hard field snap (set_relative_odom).
_RESET_VAR = 1.0


def _rotate_cov(pxx, pxy, pyy, r00, r01, r10, r11):
    """Return R P R^T for a 2x2 row-major R."""
    m00 = r00 * pxx + r01 * pxy
    m01 = r00 * pxy + r01 * pyy
    m10 = r10 * pxx + r11 * pxy
    m11 = r10 * pxy + r11 * pyy
    n00 = m00 * r00 + m01 * r01
    n01 = m00 * r10 + m01 * r11
    n10 = m10 * r00 + m11 * r01
    n11 = m10 * r10 + m11 * r11
    return n00, 0.5 * (n01 + n10), n11


class PoseEKF:
    """Position filter on absolute (odom_x, odom_y), centimetres.

    Heading stays on the BNO08x and is an input to the motion model.
    Predict integrates the encoder the same way the old odometry did.
    Updates run in the lane-relative frame, on whichever axes the sonar
    correction actually observed, then the pose is mapped back.
    """

    def __init__(self):
        self.pxx = _RESET_VAR
        self.pxy = 0.0
        self.pyy = _RESET_VAR
        self.path_cm = 0.0
        self.last_innov_x = None
        self.last_innov_y = None

    def trace(self):
        return self.pxx + self.pyy

    def reset_from_state(self):
        """Trust the pose that set_relative_odom just wrote."""
        self.pxx = _RESET_VAR
        self.pxy = 0.0
        self.pyy = _RESET_VAR
        self.path_cm = 0.0
        self.last_innov_x = None
        self.last_innov_y = None

    def _frame(self):
        """Linear map rel = R * abs + t, matching state.get_relative_odom.

        Returns (r00, r01, r10, r11, tx, ty). Every R here is orthogonal,
        so the inverse is R^T and covariance maps as R P R^T.
        """
        lane = state.relative_lane
        mirror = state.clockwise == -1
        wall = state.outer_wall
        if lane == 0:
            if mirror:
                return 1.0, 0.0, 0.0, 1.0, 0.0, 0.0
            return -1.0, 0.0, 0.0, 1.0, wall, 0.0
        if lane == 1:
            if mirror:
                return 0.0, 1.0, -1.0, 0.0, 0.0, wall
            return 0.0, 1.0, 1.0, 0.0, 0.0, 0.0
        if lane == 2:
            if mirror:
                return -1.0, 0.0, 0.0, -1.0, wall, wall
            return 1.0, 0.0, 0.0, -1.0, 0.0, wall
        if mirror:
            return 0.0, -1.0, 1.0, 0.0, wall, 0.0
        return 0.0, -1.0, -1.0, 0.0, wall, wall

    def _to_relative(self, ax, ay, pxx, pxy, pyy):
        r00, r01, r10, r11, tx, ty = self._frame()
        rx = r00 * ax + r01 * ay + tx
        ry = r10 * ax + r11 * ay + ty
        qxx, qxy, qyy = _rotate_cov(pxx, pxy, pyy, r00, r01, r10, r11)
        return rx, ry, qxx, qxy, qyy

    def _to_absolute(self, rx, ry, pxx, pxy, pyy):
        r00, r01, r10, r11, tx, ty = self._frame()
        dx = rx - tx
        dy = ry - ty
        ax = r00 * dx + r10 * dy
        ay = r01 * dx + r11 * dy
        qxx, qxy, qyy = _rotate_cov(pxx, pxy, pyy, r00, r10, r01, r11)
        return ax, ay, qxx, qxy, qyy

    def predict(self, ds_cm, theta_rad):
        """Integrate one encoder step. theta_rad is the compass heading.

        Travel direction is the same mirrored heading the old integrator
        used (clockwise flips both axes). Process noise is accumulated in
        that direction: along-track from encoder scale error, cross-track
        from heading error.

        Both of those are biases over the path, so the variance has to grow
        with distance squared. Adding (k * ds)^2 on every 10 ms tick would
        shrink that by the number of ticks. path_cm makes one 10 cm step and
        ten 1 cm steps write the same covariance. The 0.05 cm term is
        uncorrelated count noise and grows with distance, not with the
        square. A step that does not move the pose (stopped, or
        clockwise == 0) leaves the covariance alone.
        """
        cw = state.clockwise
        dx = ds_cm * math.cos(theta_rad) * cw * -1.0
        dy = ds_cm * math.sin(theta_rad) * cw
        if dx == 0.0 and dy == 0.0:
            return

        state.odom_x += dx
        state.odom_y += dy

        dist = math.sqrt(dx * dx + dy * dy)
        ux = dx / dist
        uy = dy / dist
        step = abs(ds_cm)
        travelled = self.path_cm
        travelled_next = travelled + step
        span = travelled_next * travelled_next - travelled * travelled
        var_along = (_ALONG_SCALE * _ALONG_SCALE) * span + (_ALONG_FLOOR_CM * _ALONG_FLOOR_CM) * step
        var_cross = (_HEADING_SIGMA_RAD * _HEADING_SIGMA_RAD) * span
        self.path_cm = travelled_next
        self.pxx += var_along * ux * ux + var_cross * uy * uy
        self.pxy += (var_along - var_cross) * ux * uy
        self.pyy += var_along * uy * uy + var_cross * ux * ux

    def update_relative(self, z_x, z_y, observe_x, observe_y):
        """Kalman update from a lane-relative sonar position.

        observe_x / observe_y select which components of (z_x, z_y) are
        real measurements. The other component is ignored, so a corridor
        fix cannot pull the unobserved axis. Each axis is gated on its
        own. Returns (innov_x, innov_y, accepted_x, accepted_y).
        """
        ax = state.odom_x
        ay = state.odom_y
        rx, ry, pxx, pxy, pyy = self._to_relative(ax, ay, self.pxx, self.pxy, self.pyy)

        innov_x = None
        innov_y = None
        accepted_x = False
        accepted_y = False

        if observe_x:
            innov_x = z_x - rx
            rx, ry, pxx, pxy, pyy, accepted_x = _update_axis(
                rx, ry, pxx, pxy, pyy, innov_x, 0
            )
        if observe_y:
            innov_y = z_y - ry
            rx, ry, pxx, pxy, pyy, accepted_y = _update_axis(
                rx, ry, pxx, pxy, pyy, innov_y, 1
            )

        if accepted_x or accepted_y:
            ax, ay, pxx, pxy, pyy = self._to_absolute(rx, ry, pxx, pxy, pyy)
            state.odom_x = ax
            state.odom_y = ay
            self.pxx = pxx
            self.pxy = pxy
            self.pyy = pyy
            # Position was corrected. Further scale and heading error is
            # measured from here, not from the start of the run.
            self.path_cm = 0.0

        self.last_innov_x = innov_x
        self.last_innov_y = innov_y
        return innov_x, innov_y, accepted_x, accepted_y


def _update_axis(x, y, pxx, pxy, pyy, innov, axis):
    """Joseph-form scalar update. axis 0 observes x, axis 1 observes y."""
    if axis == 0:
        variance = pxx
        gx = pxx
        gy = pxy
    else:
        variance = pyy
        gx = pxy
        gy = pyy

    innovation_var = variance + _SONAR_VAR
    if innovation_var <= 0.0 or innov * innov > _GATE * innovation_var:
        return x, y, pxx, pxy, pyy, False

    gx /= innovation_var
    gy /= innovation_var
    x += gx * innov
    y += gy * innov

    if axis == 0:
        a00 = 1.0 - gx
        a01 = 0.0
        a10 = -gy
        a11 = 1.0
    else:
        a00 = 1.0
        a01 = -gx
        a10 = 0.0
        a11 = 1.0 - gy

    m00 = a00 * pxx + a01 * pxy
    m01 = a00 * pxy + a01 * pyy
    m10 = a10 * pxx + a11 * pxy
    m11 = a10 * pxy + a11 * pyy
    n00 = m00 * a00 + m01 * a01
    n01 = m00 * a10 + m01 * a11
    n10 = m10 * a00 + m11 * a01
    n11 = m10 * a10 + m11 * a11
    n00 += gx * _SONAR_VAR * gx
    n11 += gy * _SONAR_VAR * gy
    cross = gx * _SONAR_VAR * gy
    n01 += cross
    n10 += cross
    return x, y, n00, 0.5 * (n01 + n10), n11, True


pose_ekf = PoseEKF()
