import sensor, image, time, pyb, math
from machine import LED


# Fixed exposure in microseconds. Auto exposure is disabled: it left the image
# brighter on some power-ups, and every brightness change moves the colour
# thresholds.
#
# 14 000 us, not 20 000: at 20 ms the white field sat at 96 % of full scale with
# 22 % of the frame clipped, a red sign read as washed pink, and its top edge
# merged into the saturated background - which pulled the fitted height down to
# 5.0 cm, i.e. straight into the Surprise Rule band, so a normal red sign had its
# colour inverted. Aim the white field at ~200/255, not 245.
EXPOSURE_US = 20000

# What counts as "the exposure lock took": the sensor quantises the value to
# rows, so the readback is never exactly EXPOSURE_US.
EXPOSURE_TOLERANCE_US = 300

# Safety net for the same failure: if the pixels just above a blob are blown out,
# the blob's top edge is not the sign's top edge, and the height fit is a guess
# the Surprise Rule will then act on (a normal 10 cm sign read as 5.0 inverts its
# colour and the car takes the wrong side). Reporting the height as unknown (0.0)
# instead costs at most the Surprise bonus on that sign; inverting wrongly costs
# the side of every normal sign that reads tall. Turn it off while measuring the
# mounting constants if it fires on a properly exposed frame.
CLIP_GUARD = True
# How much brighter the strip above a blob has to be than the blob's own top
# rows for the boundary to count as a real edge. A sign standing on a lit floor
# gives tens of L; a boundary that is only the edge of the clip gives a few.
CLIP_MIN_CONTRAST_L = 25

DEBUG = False
GREEN_THRESHOLDS = [(37, 86, -54, -19, -22, 37)]
RED_THRESHOLDS = [(27, 100, 18, 69, -127, 123)]#(23, 71, 23, 74, -30, 49)]

# GAIN: 0 dB is the noise floor. If the field looks dark at this exposure, raise
# EXPOSURE_US first - doubling it is about +6 dB, with less noise than gain.
#
# WHITE BALANCE: fixed to a NUMBER, like the other two. The camera cannot derive
# it from the boot scene - powered up facing a black wall there is no white to
# average, the tuple stays (0, 0, 0) and everything afterwards reads yellow - so
# the number has to come from one measurement under the light the car will run in.
# wb_probe.py prints it in paste-ready form: point the camera at the field (or a
# sheet of white paper on it), run that script, and copy its FIXED_RGB_GAINS line
# here. From then on the image is identical whatever the camera is pointing at
# when it is switched on. (0, 0, 0) is the sensor's own balance: fixed, so
# repeatable, but it keeps the native warm tint - use it only until the real
# number is measured.
FIXED_GAIN_DB = 0.0
FIXED_RGB_GAINS = (0.0, 0.0, 0.0)
AUTO_WHITEBAL = False

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
#
# Refit 2026-11-25, twice, and the two fits DO NOT AGREE - the camera moved
# between them by ~0.7 deg of pitch:
#   poses 73/94/120 cm (10 cm sign)  -> 9.80 / 7.68, rms 0.46 px
#   poses 33/133/172 cm (10 cm signs) -> 9.34 / 8.48, rms 2.13 px
#   all six together                  -> 9.10 / 7.76, rms 3.37 px (no compromise)
# The values here are the second set, because those are the measurements taken
# against the camera as it stands now: with the first set the same three signs
# came back 189.7 / 35.3 / 215.3 cm against taped 133 / 33 / 172 (+57, +2, +43),
# and with these they come back 144.6 / 32.1 / 159.7 (+12, -1, -12).
#
# What that costs is worth knowing before trusting the numbers: at 1.7 m the
# floor-contact row moves 0.11 px per cm, so ONE pixel of blob edge is 9 cm of
# distance, and the residual of these poses is ~2 px. Past ~1 m the camera's Y
# is a range estimate of +-10-20 cm, not a measurement. The match radius in
# camera.py is 10 cm, so a sign read from further than that is placed roughly.
#
# The fix is mechanical, not here: the mount has to be rigid, and --self-test
# has to be re-run after anything that can move the camera, because 0.7 deg of
# drift is 40 cm of distance error at 2 m.
CAM_HEIGHT_CM = 9.34             # lens above the floor
CAM_PITCH_DEG = 8.48             # optical axis below horizontal
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
# 20, not 18: this gate is a sanity check on the fit, and the bisection bracket
# tops out at 20 (solve_height's lo, hi = 2.0, 20.0). With 18 a 15 cm sign whose
# fit came back high - which is what a close, clipped top edge produces - was
# `continue`d straight out of the loop and vanished from the CAM line entirely,
# with nothing in the log to say so. At 20 the blob survives and snap_height()
# decides: if no legal height (5/10/15) reproduces the span it reports 0.0, which
# the Surprise Rule ignores. Present-but-uncertain beats silently missing.
MAX_SIGN_HEIGHT_CM = 20.0
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
# Beyond this the height stops being measurable. The gap between a normal and a
# modified sign is 5 cm, which projects to 1575/D pixels: 26 px at 60 cm, 13 at
# 120, 10 at 150, 8 at 190. The blob's edge is uncertain by 2-4 px per side
# (measured on three 10 cm signs in one frame), so from about 180 cm on the edge
# error is the whole gap and any answer is a coin flip. Far away the colour and
# the bearing are still good; the height is not, so it is reported as unknown
# and nothing is inverted on it. The car re-reads the sign as it closes in.
# (The 190 cm misread that prompted this was really a mounting error - see
# CAM_PITCH_DEG - so this is a backstop, not the fix.)
MAX_HEIGHT_DISTANCE_CM = 180.0
# The only heights the rules allow. A fit is snapped to the nearest of them when
# it is close enough, so the firmware can key the Surprise Rule off an exact
# 5.0 / 15.0 instead of a band wide enough to catch normal 10 cm signs.
LEGAL_HEIGHTS_CM = (5.0, 10.0, 15.0)
# (below, above) in cm. The 10/15 boundary sits at 14.0, NOT at the midpoint
# 12.5, because the fit is biased TALL and the bias is measurable:
#   real 10 cm signs have come back at 12.2 - 12.6 (measured, 4 signs)
#   real 15 cm signs have come back at 15.9 - 17.5
# At 12.5 a normal green read 12.57 by 0.07 cm and was reported as a 15 cm
# Surprise, so the car inverted it and drove the wrong side of a normal sign.
# A boundary at 14.0 is 1.4 cm above the worst 10 cm read and 1.9 cm below the
# best 15 cm read, and it errs in the safe direction: a missed Surprise costs
# the side of one sign, a false one costs the side of every normal sign.
# The top of the 15 band stays 18.0 for the same reason (a 15 reads ~+17 %).
SNAP_TOLERANCE_CM = ((1.5, 1.5), (2.5, 4.0), (1.0, 3.0))
# The Surprise Rule decision, defined once. camera.py's apply_surprise_rule()
# uses the same +-0.6 around the same two heights; the LED block below used to
# carry its own 7.5 / 12.5 bands and could disagree with it on one frame (a
# 7.46 fit lit "modified" while the field kept the colour).
MODIFIED_HEIGHTS_CM = (5.0, 15.0)
MODIFIED_TOL_CM = 0.6

# --- sensor init -----------------------------------------------------------
# The three numbers that pin the image live at the top of the file: EXPOSURE_US,
# FIXED_GAIN_DB and FIXED_RGB_GAINS. "Stop adapting" is not "lock at this value":
# set_auto_gain(False) with no argument freezes whatever the last auto frame
# produced, and powering up facing a black wall used to freeze maximum gain.
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(True)
sensor.set_hmirror(True)

# The exposure lock has to be verified, not assumed. On this firmware build
# set_auto_exposure(False, exposure_us=X) returns without error and leaves the
# register at whatever the auto loop had reached - measured: wanted 10 000, the
# sensor reported 25 000 after a boot against a black wall, so the image stayed
# blown out on the field. Set, read back, retry; and keep the number that
# actually took so the log cannot lie about it.
def _lock_exposure(us, attempts=3):
    reported = None
    for _ in range(attempts):
        sensor.set_auto_exposure(False, exposure_us=us)
        sensor.skip_frames(time=100)
        try:
            reported = sensor.get_exposure_us()
        except Exception:
            return None
        if abs(reported - us) <= EXPOSURE_TOLERANCE_US:
            return reported
    return reported

_exposure_now = _lock_exposure(EXPOSURE_US)

sensor.set_auto_gain(False, gain_db=FIXED_GAIN_DB)
if FIXED_RGB_GAINS is not None:
    sensor.set_auto_whitebal(False, rgb_gain_db=FIXED_RGB_GAINS)
elif AUTO_WHITEBAL:
    # Let it converge on whatever the camera is actually looking at, then it
    # holds: the field light does not change between the start and the parking.
    sensor.set_auto_whitebal(True)
else:
    sensor.set_auto_whitebal(False)
# AWB needs a few frames of the real scene before the wb= it reports means
# anything; the exposure and gain are already fixed, so this wait costs nothing.
sensor.skip_frames(time=800 if AUTO_WHITEBAL and FIXED_RGB_GAINS is None else 300)
print("Camera locked: exposure=%d us, gain=%.1f dB, wb=%s"
      % (EXPOSURE_US, sensor.get_gain_db(), sensor.get_rgb_gain_db()))

# Boot probe. Three samples of the same scene: if they drift apart, something is
# still adapting even though the three loops were told to stop. The exposure
# readback is what proves set_auto_exposure took; when it does not, exposure_us
# is the number the sensor is really integrating, and a dark boot scene (a black
# wall) is what chose it.
#
# l_mean is a plain int on this firmware build and a method on newer ones, the
# same attribute-or-method split that bit the blob fields; so is get_exposure_us
# on some builds, hence the guards.
def _luma(img):
    value = img.get_statistics().l_mean
    return value() if callable(value) else value

_lumas = []
for _ in range(3):
    _lumas.append(_luma(sensor.snapshot()))
    sensor.skip_frames(time=150)

print("boot probe: exposure=%s us (wanted %d), gain=%.1f dB, wb=%s, luma=%s"
      % (_exposure_now, EXPOSURE_US, sensor.get_gain_db(), sensor.get_rgb_gain_db(),
         ",".join("%.1f" % v for v in _lumas)))


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
def _drop(frame_no, reason, color, by, bh, bw, dens, D, h_raw):
    """Say why a blob was thrown away.

    The three `continue`s below used to be silent, and that made a missing sign
    undiagnosable: with four signs in front, the 15 cm red simply did not appear
    in the CAM line, and nothing in the log distinguished "find_blobs never saw
    it" from "the aspect filter ate it" from "the height fit landed outside the
    gate". Throttled like every other debug line so the USB buffer keeps up.
    """
    if not DEBUG or (frame_no % 10) != 0:
        return
    print("DROP:%s color=%s rows=%d..%d w=%d dens=%.2f D=%.1f h_raw=%.2f"
          % (reason, color, by, by + bh, bw, dens, D, h_raw))


def _blob_field(blob, name):
    """Read a blob field whether this OpenMV build exposes it as an
    attribute or as a method (blob.code vs blob.code())."""
    value = getattr(blob, name)
    return value() if callable(value) else value


def _roi_l(img, x, y, w, h):
    """Mean luminance of a rectangle, or None if the statistics refuse."""
    if w < 1 or h < 1 or x < 0 or y < 0 or x + w > W or y + h > H:
        return None
    try:
        stats = img.get_statistics(roi=(x, y, w, h))
    except Exception:
        return None
    value = stats.l_mean
    return value() if callable(value) else value


def _edge_blown(img, bx, by, bw, bh):
    """True when the blob's top edge is not an edge, just the edge of the clip.

    A blob whose top is against saturated pixels may not own that edge: the real
    sign boundary was eaten by the clip, so the span is short and the fitted
    height comes out low. At a 20 ms exposure a 10 cm red sign fitted 5.0 cm, and
    5.0 is the Surprise Rule signal, so its colour was inverted.

    "The strip above is white" is not the test. A red 5 cm sign at 46 cm has the
    lit floor right above its top, that strip is blown, and it still has a real
    edge - the first version of this guard reported h=0.0 for exactly that sign.
    What actually separates the two cases is CONTRAST across the boundary: a sign
    against a blown floor goes from dark to bright (tens of L), while a boundary
    produced by the clip goes from bright to bright (a few L).

    Measured as mean luminance of small ROIs, not pixel by pixel: get_pixel()
    returns the packed RGB565 integer on this build, so unpacking it here would
    be a second encoding to keep right.
    """
    if bh < 4 or bw < 4:
        return False

    above = _roi_l(img, bx, by - 2, bw, 1)
    # The blob's own top fifth: if the clip ate the sign's top, these rows are
    # already as bright as the strip above them.
    inside = _roi_l(img, bx, by, bw, max(2, bh // 5))
    if above is None or inside is None:
        return False
    if above < 90:                      # nothing blown above: edge is real
        return False
    return (above - inside) < CLIP_MIN_CONTRAST_L



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
    """Nearest legal sign height, or 0.0 when the fit is no legal height at all.

    0.0 means UNKNOWN, exactly like a clipped blob, and it is what goes on the
    wire: the controller keeps the colour it saw and never inverts on it. Until
    this change the raw value went out instead, so 7.46 was sent as a 7.5 cm
    sign. No legal sign is 7.5 cm: the controller's +-0.6 band kept the colour,
    but the on-board LED decision (h < 7.5) lit "modified", and the two
    disagreed on the same frame.

    The gap is not a corner case. The vertical fit is 20-25 % uncertain (the
    distance and the height scale together, see CAM_PITCH_DEG), so a normal
    10 cm sign can land anywhere in 7.5..12.5 and only the snap decides which
    side of the boundary it falls on.
    """
    for legal, tol in zip(LEGAL_HEIGHTS_CM, SNAP_TOLERANCE_CM):
        if -tol[0] <= hcm - legal <= tol[1]:
            return legal
    return 0.0


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
            _drop(frame_no, "density", color, by, bh, bw, dens, 0.0, 0.0)
            continue
        if bh < bw * MIN_ASPECT:
            _drop(frame_no, "aspect", color, by, bh, bw, dens, 0.0, 0.0)
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
        # d_width is a distance to the column CENTRE, like every other D in this
        # file (solve_dh models Z = D +- SIGN_HALF). Without the + SIGN_HALF it
        # was the distance to the front face, so whenever the guard re-ranged,
        # it under-ranged by 2.5 cm: 4.4 % at 56 cm, 1.7 % at 150 cm, and the
        # height re-solved there came out short by the same fraction. It also
        # uses the LATERAL focal, the one that measures widths: seeding with
        # FOCAL_PX here left two different focals in the same horizontal range
        # (339 vs 315, 7 %).
        w_px = bw if bw > 4 else 4
        d_width = FOCAL_LAT_PX * (2.0 * SIGN_HALF) / w_px + SIGN_HALF
        D0 = d_width
        dh = solve_dh(by, by + bh, D0)
        if dh is None:
            _drop(frame_no, "no_fit", color, by, bh, bw, dens, D0, 0.0)
            continue
        D, hcm = dh
        h_raw = hcm                         # the vertical fit, before ranging
        if d_width > 1.0 and (D > D_WIDTH_MAX_RATIO * d_width
                              or D * D_WIDTH_MAX_RATIO < d_width):
            # LOG LINE ONLY. This used to re-range on the width and report the
            # height as unknown, to catch a mounting error that slid (D,H) along
            # its 1.53x family. Two things killed that idea:
            #   1. after the 2026-11-25 refit the mounting reproduces three
            #      taped distances, so the vertical fit is the good cue;
            #   2. the width is the bad one - it came out 1.5-1.8x too wide in
            #      9 of 16 bench frames, and 1.66x here.
            # This frame is the proof that re-ranging costs a real sign: a 15 cm
            # sign at 49 cm (rows 35..141, 56 px wide) fits (50.1 cm, 15.9 cm)
            # exactly, but the width said 32.8 cm - which for a legal sign means
            # 10 cm, and a 10 cm top projects at row 75 whatever the distance.
            # The blob's top is at 35. Re-ranging threw the sign away AND its
            # Surprise inversion. Keep the fit; keep this as a trace instead.
            if DEBUG and (frame_no % 10) == 0:
                print("WIDE:D=%.1f,d_width=%.1f,ratio=%.2f"
                      % (D, d_width, D / d_width))
        if D > MAX_HEIGHT_DISTANCE_CM:
            hcm = 0.0                       # too far to resolve 10 vs 15 cm
            if DEBUG and (frame_no % 10) == 0:
                print("FAR:D=%.1f,span=%d,would_be_H=%.2f" % (D, bh, h_raw))
        else:
            hcm = snap_height(hcm)          # 0.0 = no legal height -> unknown
        if hcm == 0.0 and D <= MAX_HEIGHT_DISTANCE_CM and DEBUG and (frame_no % 10) == 0:
            # The wire says 0.0 (unknown) but the bench needs the number the
            # fit actually produced: 17.75 cm is a very different story from
            # 13.0 cm, and both come out as "unknown" on the CAM line.
            print("UNSNAPPED:D=%.1f,H_raw=%.2f" % (D, h_raw))
        solved += 1
        u_center = bx + bw * 0.5
        x = LATERAL_SIGN * (u_center - CX0) * D / FOCAL_LAT_PX
        # The top edge is the one that sets the height: if it is not the sign's
        # real edge, the fit below is a guess that the Surprise Rule would act
        # on. The floor under the blob is expected to be bright, so only the top
        # is guarded, and only when there is no contrast across it.
        clip_above = CLIP_GUARD and _edge_blown(img, bx, by, bw, bh)
        if clip_above and DEBUG and (frame_no % 10) == 0:
            print("CLIP:color=%s rows=%d..%d w=%d D=%.1f h_raw=%.2f"
                  % (color, by, by + bh, bw, D, h_raw))
        if v_clipped or clip_above:
            hcm = 0.0                       # height unknown (top cut / blown)
        elif h_raw < MIN_SIGN_HEIGHT_CM or h_raw > MAX_SIGN_HEIGHT_CM:
            _drop(frame_no, "height", color, by, bh, bw, dens, D, h_raw)
            continue                        # not a sign-sized blob at all
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
    # Same rule as apply_surprise_rule() in camera.py: invert on a confident
    # 5.0 or 15.0, never on an unknown (0.0) or on a normal 10.0.
    is_modified = (abs(nearest_h - MODIFIED_HEIGHTS_CM[0]) <= MODIFIED_TOL_CM
                   or abs(nearest_h - MODIFIED_HEIGHTS_CM[1]) <= MODIFIED_TOL_CM)
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
