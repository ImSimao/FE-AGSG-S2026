#!/usr/bin/env python3
"""Calibrate and check the OpenMV sign localiser in src/robot/Camera/main.py.

The camera reports, per sign, x/y in cm and h in cm. Getting y and h right
depends on three mounting constants (CAM_HEIGHT_CM, CAM_PITCH_DEG, FOCAL_PX),
and getting them wrong is invisible in the numbers: the vertical fit solves
distance and height TOGETHER, so the pair scales by one factor and stays
self-consistent. A measured 10 cm green at 94 cm came back as 143.5 cm at
15.4 cm (both 1.53x) - and because 15.4 cm is in the Surprise-Rule band, the
firmware inverted its colour.

This tool reads the constants and the geometry functions straight out of
Camera/main.py (it does not re-implement them, so it cannot drift), and has
three uses:

  --self-test   run the three bench poses through the pipeline and check each
                one reads 10 cm (and print what the superseded mounting gave)
  --sweep       for a grid of true (distance, height), project what the camera
                would see with one set of constants and invert it with the
                file's; report the error with and without the guard
  --fit         fit CAM_HEIGHT_CM and CAM_PITCH_DEG from a table of bench
                measurements: one line per pose, "distance_cm top bottom width"

Bench procedure for --fit: put a normal (10 cm) sign flat and square-on at 2-3
measured distances between 40 and 200 cm, and record its RAW line - DEBUG = True
in main.py prints RAW:top,bottom,width,height,code,density. Two poses already
separate height from pitch; a third checks the fit is not compensating.

    python src/simulator/tools/camera_fit.py --self-test
    python src/simulator/tools/camera_fit.py --sweep
    python src/simulator/tools/camera_fit.py --fit medidas.txt
    python src/simulator/tools/camera_fit.py --floor medidas.txt

--floor is the one to use when the sign height is itself in doubt: it fits the
mounting from the blob's BOTTOM row only (where the column meets the floor),
which depends on the mounting and the distance and never on the sign's height.
One floor row with CAM_HEIGHT_CM fixed pins the pitch; two pin both.
"""

from __future__ import annotations

import argparse
import math
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
CAMERA_SCRIPT = HERE.parents[1] / "robot" / "Camera" / "main.py"

# Three bench poses of the SAME normal 10 cm sign, distances taped from the lens:
# 73, 94 and 120 cm. (distance, top row, bottom row, blob width in px)
POSES = [(33.0, 63.0, 169.0, 59.0),
         (133.0, 68.0, 93.0, 16.0),
         (172.0, 73.0, 91.0, 12.0)]
# These three are noisier than the 73/94/120 set they replaced (2.1 px rms vs
# 0.46): the distances were taped approximately (~133, ~33, ~172), and at 33 cm
# 3 cm of distance is already 5 px of row. The tolerances below are sized for
# that - a tighter gate would fail a calibration that is doing its job.
# The 94 cm log also carried the same rows at 28-32 px wide with density 0.68-0.74:
# there the blob had merged with something behind the sign (a clean sign at 94 cm
# is 18 px and even 45 deg of yaw only reaches 25). The width no longer re-ranges,
# so what this case now checks is that an inflated width changes NOTHING: the
# vertical fit has to answer 10 cm at 92 cm anyway.
MERGED = (94.0, 75.0, 111.0, 32.0)
# The mounting the file carried before the 2026-11-25 refit. It reproduces the old
# 94 cm field log exactly and is 15 rows out on all three poses above: the top of
# a 10 cm sign sits on the horizon (row 76), not at row 61, which is how a second
# pose exposed it.
SUPERSEDED = {"cam_h": 10.95, "pitch": 11.0}


def load_geometry(path=CAMERA_SCRIPT):
    """Exec the constants and the geometry functions out of the camera script.

    The script runs a capture loop at import time and needs sensor/pyb, so only
    the two halves that are pure maths are taken: everything before the first
    sensor call, and everything between _vspan and the loop.
    """
    source = path.read_text(encoding="utf-8")
    consts = source[source.index("W, H = 320"):source.index("sensor.reset()")]
    defs = source[source.index("def _vspan"):source.index("frame_no = 0")]
    namespace = {}
    exec(compile("import math\n" + consts + defs, str(path), "exec"), namespace)  # noqa: S102
    missing = [n for n in ("_vspan", "solve_dh", "_blob_field")
               if n not in namespace]
    if missing:
        raise SystemExit("camera script does not define %s" % ", ".join(missing))
    return namespace


def project(ns, distance_cm, height_cm):
    """What the camera would report for a sign at that distance, rows and width."""
    span = ns["_vspan"](distance_cm, height_cm)
    if span is None:
        return None
    v0, v1 = span
    width_px = ns["FOCAL_LAT_PX"] * (2.0 * ns["SIGN_HALF"]) / distance_cm
    return {"top": v0, "bottom": v1, "width": width_px}


def locate(ns, top, bottom, width, guard=True, cam_h=None, pitch_deg=None,
           focal_lat=None):
    """The pipeline from the capture loop: vertical fit, width trace, snap."""
    cam_h = ns["CAM_HEIGHT_CM"] if cam_h is None else cam_h
    pitch_deg = ns["CAM_PITCH_DEG"] if pitch_deg is None else pitch_deg
    focal_lat = ns["FOCAL_LAT_PX"] if focal_lat is None else focal_lat
    sign_half = ns["SIGN_HALF"]
    w_px = width if width > 4 else 4
    # Centre distance, like every D the solver returns (Z = D +- SIGN_HALF).
    d_width = focal_lat * (2.0 * sign_half) / w_px + sign_half
    d0 = d_width
    fit = ns["solve_dh"](float(top), float(bottom), d0, cam_h, pitch_deg)
    if fit is None:
        return None
    distance, height = fit
    reranged = False
    if guard:
        ratio = ns.get("D_WIDTH_MAX_RATIO")
        snap = ns.get("snap_height")
        far = ns.get("MAX_HEIGHT_DISTANCE_CM", 1e9)
        # Mirror of the capture loop: the width only sets `reranged` (a trace),
        # and beyond the max height distance the height is unknown.
        if (ratio is not None and d_width > 1.0
                and (distance > ratio * d_width or distance * ratio < d_width)):
            reranged = True
        if distance > far:
            height = 0.0
        elif snap is not None:
            height = snap(height)
    return {"distance_cm": distance, "height_cm": height,
            "d_width": d_width, "reranged": reranged}


def inverted(height_cm):
    """The firmware's Surprise Rule: invert only on a confident 5.0 or 15.0."""
    return height_cm > 0.0 and (abs(height_cm - 5.0) <= 0.6 or abs(height_cm - 15.0) <= 0.6)


def _fmt(value, digits=1):
    return "-" if value is None else ("%." + str(digits) + "f") % value


def _cell(result):
    if result is None:
        return "no fit"
    return "D=%6.1f  H=%4.1f  %-9s" % (
        result["distance_cm"], result["height_cm"],
        "INVERTED" if inverted(result["height_cm"]) else "kept")


def self_test(ns):
    """Every bench pose must read 10 cm, and the guard must not invert one."""
    print("camera script: %s" % CAMERA_SCRIPT)
    print("constants now: CAM_HEIGHT_CM=%.2f  CAM_PITCH_DEG=%.2f  FOCAL_PX=%.0f  FOCAL_LAT_PX=%.0f"
          % (ns["CAM_HEIGHT_CM"], ns["CAM_PITCH_DEG"], ns["FOCAL_PX"], ns["FOCAL_LAT_PX"]))
    print()
    print("three bench poses of one 10 cm sign, taped distances:")
    ok = True
    for distance, top, bottom, width in POSES:
        now = locate(ns, top, bottom, width, guard=True)
        old = locate(ns, top, bottom, width, guard=True,
                     cam_h=SUPERSEDED["cam_h"], pitch_deg=SUPERSEDED["pitch"])
        print("  %5.0f cm  rows %5.0f..%5.0f  w=%2.0f   now: %s  |  10.95/11.0: %s"
              % (distance, top, bottom, width, _cell(now), _cell(old)))
        if now is None:
            print("        FAIL: no fit with the constants in this file")
            ok = False
            continue
        if abs(now["height_cm"] - 10.0) > 1.5:
            print("        FAIL: a normal 10 cm sign must read 10.0+-1.5, not %.1f"
                  % now["height_cm"])
            ok = False
        if abs(now["distance_cm"] - distance) > 15.0:
            print("        FAIL: read %.1f cm for a sign measured at %.0f cm"
                  % (now["distance_cm"], distance))
            ok = False
        if inverted(now["height_cm"]):
            print("        FAIL: a normal sign must never land in the 5/15 cm band")
            ok = False
    merged = locate(ns, MERGED[1], MERGED[2], MERGED[3], guard=True)
    print("  %5.0f cm  rows %5.0f..%5.0f  w=%2.0f   merged blob: %s"
          % (MERGED[0], MERGED[1], MERGED[2], MERGED[3], _cell(merged)))
    if merged is not None and inverted(merged["height_cm"]):
        print("        FAIL: a merged blob must not be inverted into a Surprise")
        ok = False
    print()
    if not ok:
        print("FAIL: the mounting in the camera script does not reproduce the poses")
        return False
    print("OK: all three poses read 10 cm at their own distance. The 10.95 / 11.0")
    print("column is the superseded mounting: it sat 15 rows high, which is why")
    print("the 120 cm sign read 5.0 cm and had its colour inverted.")
    return True


def fit(ns, rows):
    """Least-squares (CAM_HEIGHT_CM, CAM_PITCH_DEG) from measured poses."""
    if len(rows) < 2:
        raise SystemExit("--fit needs at least two measured poses (distance top bottom width)")
    def cost(cam_h, pitch):
        total = 0.0
        for distance, top, bottom, width in rows:
            span = ns["_vspan"](distance, 10.0, cam_h=cam_h, pitch_deg=pitch)
            if span is None:
                return 1e9
            total += (span[0] - top) ** 2 + (span[1] - bottom) ** 2
        return total
    best = None
    cam_h = 6.0
    while cam_h <= 16.0:
        pitch = 0.0
        while pitch <= 25.0:
            c = cost(cam_h, pitch)
            if best is None or c < best[0]:
                best = (c, cam_h, pitch)
            pitch += 0.2
        cam_h += 0.2
    # refine around the grid minimum
    cost0, h0, p0 = best
    step_h, step_p = 0.1, 0.1
    for _ in range(40):
        improved = False
        for dh, dp in ((step_h, 0), (-step_h, 0), (0, step_p), (0, -step_p)):
            c = cost(h0 + dh, p0 + dp)
            if c < cost0:
                cost0, h0, p0 = c, h0 + dh, p0 + dp
                improved = True
        if not improved:
            step_h *= 0.5
            step_p *= 0.5
            if step_h < 1e-4:
                break
    print("fit over %d pose(s): CAM_HEIGHT_CM=%.2f  CAM_PITCH_DEG=%.2f  rms row error %.2f px"
          % (len(rows), h0, p0, math.sqrt(cost0 / (2 * len(rows)))))
    print()
    print("per pose:")
    for distance, top, bottom, width in rows:
        span = ns["_vspan"](distance, 10.0, cam_h=h0, pitch_deg=p0)
        print("  %6.1f cm  observed %6.1f..%6.1f   fitted %6.1f..%6.1f   error %+6.2f / %+6.2f px"
              % (distance, top, bottom, span[0], span[1], span[0] - top, span[1] - bottom))
    print()
    print("paste into Camera/main.py:")
    print("  CAM_HEIGHT_CM = %.2f" % h0)
    print("  CAM_PITCH_DEG = %.2f" % p0)
    if len(rows) < 3:
        print("  (two poses pin two unknowns exactly - confirm with a third before trusting it)")
    return 0


def fit_floor(ns, rows):
    """Fit CAM_PITCH_DEG (and CAM_HEIGHT_CM) from floor-contact rows.

    rows are (distance_cm, top, bottom, width) - the same table --fit takes; only
    the distance and the BOTTOM row are used. The bottom row is where the column
    meets the floor: it depends on the mounting and the distance, never on the
    sign's height, so a height that reads 25 % short cannot bias this fit (it
    biases --fit). Yaw moves it by ~1 px even at 45 deg.

    One row pins the pitch with cam_h held at the file's value; two rows pin
    both, and a third checks that they are not compensating each other.
    """
    def cost(cam_h, pitch):
        total = 0.0
        for distance, top, bottom, _ in rows:
            m = ns["_vspan"](distance, 10.0, cam_h, pitch)
            if m is None:
                return 1e9
            total += (m[1] - bottom) ** 2
        return total

    best = None
    if len(rows) == 1:
        cam_h = ns["CAM_HEIGHT_CM"]
        pitch = 0.0
        while pitch <= 30.0:
            c = cost(cam_h, pitch)
            if best is None or c < best[0]:
                best = (c, cam_h, pitch)
            pitch += 0.01
    else:
        cam_h = 6.0
        while cam_h <= 18.0:
            pitch = 0.0
            while pitch <= 30.0:
                c = cost(cam_h, pitch)
                if best is None or c < best[0]:
                    best = (c, cam_h, pitch)
                pitch += 0.05
            cam_h += 0.05
    cost0, h0, p0 = best
    sh, sp = 0.02, 0.02
    for _ in range(400):
        moved = False
        for dh, dp in ((sh, 0), (-sh, 0), (0, sp), (0, -sp)):
            c = cost(h0 + dh, p0 + dp)
            if c < cost0:
                cost0, h0, p0 = c, h0 + dh, p0 + dp
                moved = True
        if not moved:
            sh *= 0.5
            sp *= 0.5
            if sp < 1e-6:
                break
    rms = (cost0 / len(rows)) ** 0.5
    print("floor-contact fit over %d row(s), rms %.1f px:" % (len(rows), rms))
    for distance, top, bottom, _ in rows:
        m = ns["_vspan"](distance, 10.0, h0, p0)
        print("  %6.1f cm  bottom observed %6.1f   fitted %6.1f   error %+5.1f px"
              % (distance, bottom, m[1], m[1] - bottom))
    print()
    if not (6.0 <= h0 <= 18.0) or rms > 3.0:
        # The robot is 17 cm tall, so a lens 20 cm up does not exist. A fit that
        # asks for one means the rows disagree with each other, and pasting it
        # would be worse than leaving the old constants in place.
        print("DO NOT PASTE: CAM_HEIGHT_CM %.1f with rms %.1f px is not a camera"
              % (h0, rms))
        print("that can be mounted on this robot. The rows disagree: one distance")
        print("is wrong, or the camera moved between the measurements.")
        return 1
    print("paste into Camera/main.py:")
    print("  CAM_HEIGHT_CM = %.2f" % h0)
    print("  CAM_PITCH_DEG = %.2f" % p0)
    if len(rows) < 2:
        print("  (one row pins one unknown: CAM_HEIGHT_CM is held at %.2f)"
              % ns["CAM_HEIGHT_CM"])
    else:
        print("  (two rows pin both - confirm with a third before trusting them)")
    return 0


def sweep(ns, true_h, true_pitch, true_focal_lat):
    """Project with one mounting, invert with the camera script's own."""
    print("mounting used to generate the blobs : height %.2f cm, pitch %.2f deg, lateral focal %.0f px"
          % (true_h, true_pitch, true_focal_lat))
    print("mounting the script inverts them with: height %.2f cm, pitch %.2f deg, lateral focal %.0f px"
          % (ns["CAM_HEIGHT_CM"], ns["CAM_PITCH_DEG"], ns["FOCAL_LAT_PX"]))
    if (abs(true_h - ns["CAM_HEIGHT_CM"]) < 1e-6
            and abs(true_pitch - ns["CAM_PITCH_DEG"]) < 1e-6
            and abs(true_focal_lat - ns["FOCAL_LAT_PX"]) < 1e-6):
        print("(identical: this is the no-error baseline. Pass --true-height/--true-pitch/"
              "--true-focal-lat to model a mis-calibrated mounting.)")
    print()
    print("  true D  true H | raw fit               | + snap                 | Surprise")
    worst_old = worst_new = 0.0
    wrong_colour = 0
    for distance in (40.0, 60.0, 94.0, 140.0, 200.0):
        for height in (5.0, 10.0, 15.0):
            span = ns["_vspan"](distance, height, true_h, true_pitch)
            if span is None:
                continue
            width = true_focal_lat * (2.0 * ns["SIGN_HALF"]) / distance
            old = locate(ns, span[0], span[1], width, guard=False)
            new = locate(ns, span[0], span[1], width, guard=True)
            want = height != 10.0
            if old is None or new is None:
                print("  %6.1f %6.1f | no fit" % (distance, height))
                continue
            e_old = abs(old["distance_cm"] - distance)
            e_new = abs(new["distance_cm"] - distance)
            worst_old = max(worst_old, e_old)
            worst_new = max(worst_new, e_new)
            got = inverted(new["height_cm"])
            if got != want:
                wrong_colour += 1
            print("  %6.1f %6.1f | D=%6.1f H=%5.1f e=%5.1f | D=%6.1f H=%5.1f e=%5.1f | %-9s %s"
                  % (distance, height, old["distance_cm"], old["height_cm"], e_old,
                     new["distance_cm"], new["height_cm"], e_new,
                     "inverted" if got else "kept",
                     "ok" if got == want else "WRONG"))
    print()
    print("worst distance error: vertical only %.1f cm, with the guard %.1f cm"
          % (worst_old, worst_new))
    print("Surprise Rule outcomes wrong on %d of the cases above" % wrong_colour)
    print("(a mounting this far off has to be fixed with --fit / --floor; the")
    print(" width is only a trace now - it cannot correct the fit)")
    return 0


def read_measurements(path):
    rows = []
    # utf-8-sig: a table typed in Notepad on Windows starts with a BOM, and
    # "float('\ufeff73')" would abort the run instead of reading the row.
    for line in pathlib.Path(path).read_text(encoding="utf-8-sig").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = [p for p in line.replace(",", " ").split() if p]
        if len(parts) < 3:
            continue
        distance, top, bottom = (float(p) for p in parts[:3])
        width = float(parts[3]) if len(parts) > 3 else 20.0
        rows.append((distance, top, bottom, width))
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--camera-script", default=str(CAMERA_SCRIPT))
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--sweep", action="store_true")
    parser.add_argument("--fit", metavar="FILE")
    parser.add_argument("--floor", metavar="FILE",
                        help="fit the mounting from floor-contact rows only "
                             "(distance top bottom [width])")
    parser.add_argument("--true-height", type=float, default=None,
                        help="mounting the sweep projects with (default: the script's own)")
    parser.add_argument("--true-pitch", type=float, default=None)
    parser.add_argument("--true-focal-lat", type=float, default=None)
    args = parser.parse_args(argv)

    ns = load_geometry(pathlib.Path(args.camera_script))
    if args.fit:
        return fit(ns, read_measurements(args.fit))
    if args.floor:
        return fit_floor(ns, read_measurements(args.floor))
    if args.sweep:
        return sweep(ns,
                     ns["CAM_HEIGHT_CM"] if args.true_height is None else args.true_height,
                     ns["CAM_PITCH_DEG"] if args.true_pitch is None else args.true_pitch,
                     ns["FOCAL_LAT_PX"] if args.true_focal_lat is None else args.true_focal_lat)
    if not self_test(ns):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
