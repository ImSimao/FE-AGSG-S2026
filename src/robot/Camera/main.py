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
DEBUG = False
GREEN_THRESHOLDS = [(21, 86, -54, -9, -128, 51)]
RED_THRESHOLDS = [
    (34, 71, 23, 74, -30, 49), #Light
    (37, 48, 56, 69, -128, 127) #Dark
]

THRESHOLDS = RED_THRESHOLDS + GREEN_THRESHOLDS   # red first -> bit0

# blob.code() is one bit per threshold, in THRESHOLDS order. Build the masks
# from the list lengths so adding or removing a threshold cannot silently
# mislabel a colour (with two red thresholds, bit 1 is red Dark, not green).
RED_MASK = (1 << len(RED_THRESHOLDS)) - 1
GREEN_MASK = ((1 << len(GREEN_THRESHOLDS)) - 1) << len(RED_THRESHOLDS)
# --- mounting (measure once, then constants) --------------------------------
W, H = 320, 240                  # QVGA
CAM_HEIGHT_CM = 10.18            # lens above the floor (bench-calibrated)
CAM_PITCH_DEG = 8.3              # optical axis below horizontal (bench-calibrated)
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
_SINP = math.sin(math.radians(CAM_PITCH_DEG))
_COSP = math.cos(math.radians(CAM_PITCH_DEG))
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
# Fixed exposure in microseconds. Auto exposure is disabled: it left the image
# brighter on some power-ups, and every brightness change moves the colour
# thresholds. Tune this once for the field lighting (10 000-20 000 us at QVGA).
EXPOSURE_US = 25000
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
def _vspan(D, Hs):
    """(v_top, v_bottom) rows of the 5x5xHs column at distance D, face-on.
    Only the VERTICAL extent is used. A rotated square column is barely taller
    in the image (span 117 px at 0 deg vs 121 px at 45 deg, ~3%), whereas its
    apparent WIDTH grows by up to 1.41x at 45 deg. A width-based range is
    therefore corrupted by yaw; the vertical extent is not.
    v depends only on Y and Z, never on X, so only the 4 (Y, Z) corners are
    projected -- half the work of the full 8 corners.
    """
    SP = _SINP
    CP = _COSP
    v0 = v1 = None
    for Y in (0.0, Hs):
        dy = Y - CAM_HEIGHT_CM
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
def solve_dh(v0_obs, v1_obs, D0):
    """Fit (D, H) so the projected vertical extent matches the blob rows.
    2 unknowns, 2 rows: a 2x2 Newton. Yaw-free. Returns (D, H) or None.
    """
    th = [D0, 10.0]
    def resid(t):
        m = _vspan(t[0], t[1])
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
    blobs = img.find_blobs(THRESHOLDS, roi=(0, 0, W, H), merge=False,
                           pixels_threshold=100, area_threshold=100)
    parts = []
    drawn = 0
    solved = 0
    nearest_d = None
    nearest_color = None
    nearest_h = 0.0
    for blob in blobs:
        if solved >= MAX_BLOBS_PER_FRAME:
            break
        code = _blob_field(blob, "code")
        red = bool(code & RED_MASK)
        green = bool(code & GREEN_MASK)
        if red and green:
            continue                        # matched red AND green: ignore
        if green:
            color = "GREEN"
        elif red:
            color = "RED"
        else:
            continue
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
        # initial guess for D: face-on width (good enough to seed Newton)
        w_px = bw if bw > 4 else 4
        D0 = FOCAL_PX * (2.0 * SIGN_HALF) / w_px + SIGN_HALF
        dh = solve_dh(by, by + bh, D0)
        if dh is None:
            continue
        D, hcm = dh
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
            "FPS:%d blobs:%d" % (clock.fps(), len(blobs)),
            color=(255, 255, 255)
        )
