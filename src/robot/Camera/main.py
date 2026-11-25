# Fixed exposure in microseconds. Auto exposure is disabled: it left the image
# brighter on some power-ups, and every brightness change moves the colour
# thresholds. Tune this once for the field lighting (10 000-20 000 us at QVGA).
EXPOSURE_US = 22000

DEBUG = False
GREEN_THRESHOLDS = [(48, 86, -54, -19, -22, 37)]
RED_THRESHOLDS = [(34, 71, 23, 74, -30, 49)]


import sensor, image, time, pyb, math
from machine import LED
# ---------------------------------------------------------------------------
# WRO 2026 Future Engineers - traffic sign localiser (OpenMV H7, OV7725)
#
# Reports, per detected sign, in CENTIMETRES, in the camera frame:
#     CAM:x,y,h,COLOR;x,y,h,COLOR;...
#       x = lateral offset of the sign CENTRE (+ = right)
#       y = horizontal forward distance of the sign CENTRE
#       h = sign height (5 / 10 / 15 cm -> tells 50 / 100 / 150 mm)
#       COLOR = RED or GREEN (the colour actually seen on the field)
#
# Method: a WRO sign is a 5x5 cm column of height H standing on the floor. The
# camera is mounted at CAM_HEIGHT_CM with the optical axis pitched CAM_PITCH_DEG
# below horizontal. We model the column's 8 corners, project them with the REAL
# pitch, and solve (Xc, D, H) so the projected bounding box matches the blob's.
# This handles the pitch explicitly: a tilted camera sees the column's depth, so
# the apparent-width method read a measured 150 mm sign as ~141 mm. The solver
# uses the vertical extent to pin the height and the full 3D box for distance.
# ---------------------------------------------------------------------------


# --- mounting (measure once, then constants) --------------------------------
W, H = 320, 240                  # QVGA
# Refitted 2026-11-17 from a measured pose: a normal 10 cm sign, 94 cm in front
# of the lens, blob rows 61..96 and 20 px wide (RAW:61,96,20,35,1,0.88). With
# the old 10.18 / 8.3 the same blob decoded as 143.5 cm at 15.4 cm - both 1.53x,
# because the vertical fit cannot separate distance from height and a pitch
# error slides it along that family. That 15.4 cm then tripped the Surprise Rule
# and turned a green sign into a red one.
#
# One pose pins two unknowns exactly, so these values are provisional: they
# reproduce the measurement by construction. Confirm with a second and third
# distance at the bench - src/simulator/tools/camera_fit.py --fit takes a table
# of "distance top bottom width" and prints what to paste here, and --self-test
# checks this file against the field-log pose.
CAM_HEIGHT_CM = 10.95            # lens above the floor
CAM_PITCH_DEG = 11.0             # optical axis below horizontal
FOCAL_PX = 315.0                 # QVGA focal, VERTICAL (bench-calibrated)
# The lateral focal that reproduces the measured x at the same distance. It
# differs from the vertical one by ~7%, which is lens/geometry asymmetry; using
# one value for both made y correct but shifted x.
FOCAL_LAT_PX = 339.0
SIGN_HALF = 2.5                  # half of the 5x5 cm column
CX0 = W / 2.0 - 1.0              # principal point
CY0 = H / 2.0 - 1.0
# vflip+hmirror = 180 deg rotation. The v axis is flipped; the u axis sign is
# kept consistent with the earlier width-based firmware (blob right of centre ->
# x positive) via LATERAL_SIGN. If a bench test with the sign clearly on the
# robot's RIGHT reports negative x, flip this to +1.
LATERAL_SIGN = -1.0
# validity gates
MIN_SIGN_HEIGHT_CM = 3.5
MAX_SIGN_HEIGHT_CM = 18.0
MIN_DISTANCE_CM = 8.0
MAX_DISTANCE_CM = 300.0
MIN_DENSITY = 0.45
MIN_ASPECT = 0.6
# solver limits
MAX_ITERS = 12                  # Newton converges in <=5; 12 is headroom
CONV_PX = 0.08
MAX_BLOBS_PER_FRAME = 6         # cap the work per frame so capture never starves
# A sign rotated by up to ~45 deg (yaw) can leave a few px of unavoidable bbox
# residual because the model assumes a face-on column. 3 px rejected real signs;
# 12 px accepts a yawed sign while still rejecting garbage fits.
MAX_RES_PX = 12.0
# The column is 5 cm wide whatever its height, so its apparent width is a range
# that does not move with H. Yaw can inflate that width by at most sqrt(2), so a
# vertical fit that lands beyond this factor of the width range cannot be a sign
# at that distance: it is the (D, H) family being followed after a calibration
# error, and it is re-ranged on the width instead.
D_WIDTH_MAX_RATIO = 1.45
# The only heights the rules allow. A fit is snapped to the nearest of them when
# it is close enough, so the firmware can key the Surprise Rule off an exact
# 5.0 / 15.0 instead of a band wide enough to catch normal 10 cm signs.
LEGAL_HEIGHTS_CM = (5.0, 10.0, 15.0)
SNAP_TOLERANCE_CM = (1.5, 2.5, 2.5)

# --- sensor init -----------------------------------------------------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(True)
sensor.set_hmirror(True)
# Let auto exposure/gain/white balance settle once, then LOCK all three.
# set_auto_gain(False) alone leaves the exposure free, so the image still
# brightened or darkened on some boots; the exposure has to be locked too.
sensor.skip_frames(time=1500)
sensor.set_auto_gain(False)
sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=500)
print("Camera locked: exposure=%d us, gain=%d dB" % (EXPOSURE_US, int(sensor.get_gain_db())))
clock = time.clock()
uart = pyb.UART(3, 115200, timeout_char=1000)   # TX on P4; never block forever
def _vspan(D, Hs, cam_h=CAM_HEIGHT_CM, pitch_deg=CAM_PITCH_DEG):
    """(v_top, v_bottom) rows of the 5x5xHs column at distance D, face-on.

    Only the VERTICAL extent is used. A rotated square column is barely taller
    in the image (span 117 px at 0 deg vs 121 px at 45 deg, ~3%), whereas its
    apparent WIDTH grows by up to 1.41x at 45 deg. A width-based range is
    therefore corrupted by yaw; the vertical extent is not.

    v depends only on Y and Z, never on X, so only the 4 (Y, Z) corners are
    projected -- half the work of the full 8 corners.

    cam_h / pitch_deg default to the calibrated constants; camera_fit.py
    overrides them to project what a *different* mounting would see, which is
    how a calibration error is reproduced offline.
    """
    SP = math.sin(math.radians(pitch_deg))
    CP = math.cos(math.radians(pitch_deg))
    v0 = v1 = None
    for Y in (0.0, Hs):
        dy = Y - cam_h
        for Z in (D - SIGN_HALF, D + SIGN_HALF):
            zc = -dy * SP + Z * CP
            if zc <= 1.0:
                return None
            v = CY0 - FOCAL_PX * (dy * CP + Z * SP) / zc
            if v0 is None or v < v0:
                v0 = v
            if v1 is None or v > v1:
                v1 = v
    return v0, v1
def solve_dh(v0_obs, v1_obs, D0, cam_h=CAM_HEIGHT_CM, pitch_deg=CAM_PITCH_DEG):
    """Fit (D, H) so the projected vertical extent matches the blob rows.

    2 unknowns, 2 rows: a 2x2 Newton. Yaw-free. Returns (D, H) or None.
    cam_h / pitch_deg are the mounting the fit is done against; the capture
    loop uses the calibrated constants, and camera_fit.py passes alternatives to
    measure what a mis-calibrated mounting would have reported.
    """
    th = [D0, 10.0]
    def resid(t):
        m = _vspan(t[0], t[1], cam_h, pitch_deg)
        if m is None:
            return 1e3, 1e3
        return m[0] - v0_obs, m[1] - v1_obs
    r = resid(th)
    for _ in range(MAX_ITERS):
        if abs(r[0]) < CONV_PX and abs(r[1]) < CONV_PX:
            break
        J00 = J01 = J10 = J11 = 0.0
        t2 = [th[0] + 0.3, th[1]]
        r2 = resid(t2)
        J00 = (r2[0] - r[0]) / 0.3
        J10 = (r2[1] - r[1]) / 0.3
        t2 = [th[0], th[1] + 0.3]
        r2 = resid(t2)
        J01 = (r2[0] - r[0]) / 0.3
        J11 = (r2[1] - r[1]) / 0.3
        det = J00 * J11 - J01 * J10
        if abs(det) < 1e-9:
            return None
        th[0] += (-r[0] * J11 + r[1] * J01) / det
        th[1] += (-r[1] * J00 + r[0] * J10) / det
        if th[0] < MIN_DISTANCE_CM:
            th[0] = MIN_DISTANCE_CM
        elif th[0] > MAX_DISTANCE_CM:
            th[0] = MAX_DISTANCE_CM
        if th[1] < 2.0:
            th[1] = 2.0
        elif th[1] > 20.0:
            th[1] = 20.0
        r = resid(th)
    if abs(r[0]) > MAX_RES_PX or abs(r[1]) > MAX_RES_PX:
        return None
    return th[0], th[1]
def _blob_field(blob, name):
    """Read a blob field whether this OpenMV build exposes it as an
    attribute or as a method (blob.code vs blob.code())."""
    value = getattr(blob, name)
    return value() if callable(value) else value


def solve_height(v0_obs, v1_obs, D, cam_h=CAM_HEIGHT_CM, pitch_deg=CAM_PITCH_DEG):
    """Height of the column at a KNOWN distance, from the vertical span.

    With D fixed the span is monotonic in H, so a bisection is enough - and it
    cannot wander the way the 2x2 Newton can when the first guess is poor.
    Returns None when no legal height reproduces the observed span, which is the
    honest answer for a blob that is not a sign.
    """
    span_obs = v1_obs - v0_obs
    lo, hi = 2.0, 20.0
    for _ in range(24):
        mid = 0.5 * (lo + hi)
        m = _vspan(D, mid, cam_h, pitch_deg)
        if m is None:
            return None
        if (m[1] - m[0]) > span_obs:
            hi = mid
        else:
            lo = mid
    best = 0.5 * (lo + hi)
    m = _vspan(D, best, cam_h, pitch_deg)
    if m is None or abs((m[1] - m[0]) - span_obs) > MAX_RES_PX:
        return None
    return best


def snap_height(hcm):
    """Nearest legal sign height, or the raw value when it is near none of them.

    The firmware keys the Surprise Rule off this: 5.0 and 15.0 mean "modified",
    10.0 means "normal", anything else means "not known well enough to invert a
    colour". Without the snap, a 10 cm sign read 25 % tall landed in the
    modified band and had its colour inverted.
    """
    for legal, tol in zip(LEGAL_HEIGHTS_CM, SNAP_TOLERANCE_CM):
        if abs(hcm - legal) <= tol:
            return legal
    return hcm


frame_no = 0
timeouts = 0

# Nearest-sign indicator: the on-board red/green LEDs mirror the colour of
# the closest detected sign (both off when nothing is seen).
led_red = LED("LED_RED")
led_green = LED("LED_GREEN")
led_blue = LED("LED_BLUE")

while True:
    clock.tick()
    frame_no += 1
    try:
        img = sensor.snapshot()
        timeouts = 0
    except Exception:
        # Frame capture timed out: the loop overran a frame (heavy processing
        # or the USB buffer filling up). Skip it. If it keeps happening the
        # sensor is stuck, so re-sync it rather than spin forever.
        timeouts += 1
        if timeouts > 20:
            try:
                sensor.skip_frames(time=200)
            except Exception:
                pass
            timeouts = 0
        continue
    parts = []
    drawn = 0
    solved = 0
    nearest_d = None
    nearest_color = None
    nearest_h = 0.0

    # Run find_blobs once per colour list: a blob's colour is the list it came
    # from, which is what the reference firmware did and is far less fragile
    # than decoding a multi-threshold code bitmask.
    candidates = []
    for blob in img.find_blobs(RED_THRESHOLDS, roi=(0, 0, W, H), merge=False,
                               pixels_threshold=100, area_threshold=100):
        candidates.append((blob, "RED"))
    for blob in img.find_blobs(GREEN_THRESHOLDS, roi=(0, 0, W, H), merge=False,
                               pixels_threshold=100, area_threshold=100):
        candidates.append((blob, "GREEN"))

    for blob, color in candidates:
        if solved >= MAX_BLOBS_PER_FRAME:
            break
        code = _blob_field(blob, "code")
        bx, by, bw, bh = _blob_field(blob, "rect")
        # Horizontal clip also corrupts the u-centre, but the distance and
        # height now come from the VERTICAL extent, so a horizontal clip only
        # offsets x slightly; a vertical clip is fatal (no v-span). Reject only
        # a vertical clip, and only when it prevents a fit.
        v_clipped = (by <= 0 or by + bh >= H)
        dens = _blob_field(blob, "density")
        if dens < MIN_DENSITY:
            continue
        if bh < bw * MIN_ASPECT:
            continue                        # flat -> floor line, not a sign
        # Two independent ranges, and they must agree.
        #
        # The vertical fit solves (D, H) together, but they scale together: a
        # 10 cm sign at 94 cm and a 15.4 cm sign at 143.7 cm project the same
        # span (ratio 1.53). A pitch calibration error slides the answer along
        # that family, which is how a measured 10 cm green at 94 cm came back
        # as CAM:...143.5,15.4 - too far and too tall by the same factor.
        #
        # The column is 5 cm wide whatever its height, so the apparent width is
        # a range that does not move with H, and yaw inflates it by at most
        # sqrt(2). A fit beyond that is impossible in either direction: yaw only
        # ever makes the vertical fit read *further* than the width, never
        # nearer. Re-range on the width and re-solve the height there.
        w_px = bw if bw > 4 else 4
        d_width = FOCAL_LAT_PX * (2.0 * SIGN_HALF) / w_px
        D0 = FOCAL_PX * (2.0 * SIGN_HALF) / w_px + SIGN_HALF
        dh = solve_dh(by, by + bh, D0)
        if dh is None:
            continue
        D, hcm = dh
        if d_width > 1.0 and (D > D_WIDTH_MAX_RATIO * d_width
                              or D * D_WIDTH_MAX_RATIO < d_width):
            D = d_width
            hcm = solve_height(by, by + bh, D)
            if hcm is None:
                continue
        hcm = snap_height(hcm)
        solved += 1
        u_center = bx + bw * 0.5
        x = LATERAL_SIGN * (u_center - CX0) * D / FOCAL_LAT_PX
        if v_clipped:
            hcm = 0.0                       # height unknown (top/bottom cut)
        elif hcm < MIN_SIGN_HEIGHT_CM or hcm > MAX_SIGN_HEIGHT_CM:
            continue
        if nearest_d is None or D < nearest_d:
            nearest_d = D
            nearest_color = color
            nearest_h = hcm
        if DEBUG:
            # Draw every surviving blob, light and dark alike, and print its
            # code so it is obvious which threshold (or both) matched.
            img.draw_rectangle(_blob_field(blob, "rect"), color=(255, 255, 255))
            img.draw_cross(
                (_blob_field(blob, "cx"), _blob_field(blob, "cy")),
                color=(255, 255, 255)
            )
            ty = _blob_field(blob, "y") - 12
            if ty < 0:
                ty = 0
            img.draw_string(
                (_blob_field(blob, "x"), ty),
                "c%d x%.1f y%.1f h%.1f" % (code, x, D, hcm),
                color=(255, 0, 0)
            )
            drawn += 1
        if DEBUG and (frame_no % 10) == 0:
            # RAW pixel bbox for calibration (throttled: printing every frame
            # can fill the USB buffer and block when nothing is reading it,
            # which is what makes snapshot() time out).
            #   RAW:top,bottom,width,height,code,density
            print("RAW:%d,%d,%d,%d,%d,%.2f" %
                  (by, by + bh, bw, bh, code, dens))
        parts.append("%.1f,%.1f,%.1f,%s" % (x, D, hcm, color))
    is_modified = nearest_h != 0.0 and (nearest_h < 7.5 or nearest_h > 12.5)
    if nearest_color is None:
        led_red.off()
        led_green.off()
        led_blue.off()
    elif nearest_color == "RED":
        led_red.on()
        if is_modified:
            led_green.on()      # red + green = yellow: 5/15 cm red sign
        else:
            led_green.off()
        led_blue.off()
    else:                       # GREEN
        if is_modified:
            led_blue.on()       # blue: 5/15 cm green sign
            led_red.off()
            led_green.off()
        else:
            led_green.on()
            led_red.off()
            led_blue.off()
    line = ("CAM:" + ";".join(parts) + "\n") if parts else "CAM:\n"
    try:
        uart.write(line)
    except Exception:
        pass                        # a stalled UART must not kill the loop
    if DEBUG:
        if (frame_no % 10) == 0:
            print(line, end="")     # echo to USB (throttled)
        img.draw_string(
            (4, 6),
            "FPS:%d blobs:%d" % (clock.fps(), len(candidates)),
            color=(255, 255, 255)
        )
