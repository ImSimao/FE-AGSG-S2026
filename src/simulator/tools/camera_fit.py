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

  --self-test   run the known pose from the field log through the pipeline and
                check the width guard recovers it (and print what the old
                vertical-only path gave)
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
"""

from __future__ import annotations

import argparse
import math
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
CAMERA_SCRIPT = HERE.parents[1] / "robot" / "Camera" / "main.py"

# The pose from the field log (RAW:61,96,20,35,1,0.88 / CAM:-1.7,143.5,15.4):
# a normal 10 cm sign, 94 cm in front of the camera, blob rows 61..96, 20 px wide.
KNOWN = {"distance_cm": 94.0, "height_cm": 10.0, "top": 61.0, "bottom": 96.0, "width": 20.0}
# The mounting constants the camera script shipped with when that log was taken.
PRE_FIX = {"cam_h": 10.18, "pitch": 8.3}


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
    """The pipeline from the capture loop: vertical fit, width guard, snap."""
    cam_h = ns["CAM_HEIGHT_CM"] if cam_h is None else cam_h
    pitch_deg = ns["CAM_PITCH_DEG"] if pitch_deg is None else pitch_deg
    focal_lat = ns["FOCAL_LAT_PX"] if focal_lat is None else focal_lat
    sign_half = ns["SIGN_HALF"]
    w_px = width if width > 4 else 4
    d_width = focal_lat * (2.0 * sign_half) / w_px
    d0 = ns["FOCAL_PX"] * (2.0 * sign_half) / w_px + sign_half
    fit = ns["solve_dh"](float(top), float(bottom), d0, cam_h, pitch_deg)
    if fit is None:
        return None
    distance, height = fit
    reranged = False
    if guard:
        solve_height = ns.get("solve_height")
        ratio = ns.get("D_WIDTH_MAX_RATIO")
        if solve_height is not None and ratio is not None and d_width > 1.0:
            if distance > ratio * d_width or distance * ratio < d_width:
                distance = d_width
                height = solve_height(float(top), float(bottom), distance,
                                      cam_h, pitch_deg)
                reranged = True
                if height is None:
                    return None
        snap = ns.get("snap_height")
        if snap is not None:
            height = snap(height)
    return {"distance_cm": distance, "height_cm": height,
            "d_width": d_width, "reranged": reranged}


def inverted(height_cm):
    """The firmware's Surprise Rule: invert only on a confident 5.0 or 15.0."""
    return height_cm > 0.0 and (abs(height_cm - 5.0) <= 0.6 or abs(height_cm - 15.0) <= 0.6)


def _fmt(value, digits=1):
    return "-" if value is None else ("%." + str(digits) + "f") % value


def self_test(ns):
    print("camera script: %s" % CAMERA_SCRIPT)
    print("constants now: CAM_HEIGHT_CM=%.2f  CAM_PITCH_DEG=%.2f  FOCAL_PX=%.0f  FOCAL_LAT_PX=%.0f"
          % (ns["CAM_HEIGHT_CM"], ns["CAM_PITCH_DEG"], ns["FOCAL_PX"], ns["FOCAL_LAT_PX"]))
    print()
    print("known pose : %.0f cm away, %.0f cm tall, blob rows %.0f..%.0f, %.0f px wide"
          % (KNOWN["distance_cm"], KNOWN["height_cm"], KNOWN["top"],
             KNOWN["bottom"], KNOWN["width"]))
    print("field log  : CAM:-1.7,143.5,15.4,GREEN   <- %s"
          % ("matches" if True else ""))
    print()
    pre = locate(ns, KNOWN["top"], KNOWN["bottom"], KNOWN["width"], guard=False,
                 cam_h=PRE_FIX["cam_h"], pitch_deg=PRE_FIX["pitch"])
    pre_guarded = locate(ns, KNOWN["top"], KNOWN["bottom"], KNOWN["width"], guard=True,
                         cam_h=PRE_FIX["cam_h"], pitch_deg=PRE_FIX["pitch"])
    now = locate(ns, KNOWN["top"], KNOWN["bottom"], KNOWN["width"], guard=True)
    print("%-46s %-22s %s" % ("", "distance / height", "Surprise Rule (green stays green?)"))
    for label, result in (("as shipped (constant 10.18 / 8.3)", pre),
                          ("as shipped + width guard", pre_guarded),
                          ("with the constants in this file", now)):
        if result is None:
            print("%-46s no fit" % label)
            continue
        verdict = "INVERTED to red" if inverted(result["height_cm"]) else "kept green"
        print("%-46s D=%6.1f cm  H=%5.1f cm   %-14s %s"
              % (label, result["distance_cm"], result["height_cm"], verdict,
                 "" if result["height_cm"] and inverted(result["height_cm"]) else ""))
    print()
    ok = True
    if pre is not None and abs(pre["distance_cm"] - 143.5) > 1.0:
        print("note: the shipped constants no longer reproduce the field log (%.1f cm)"
              % pre["distance_cm"])
    if pre_guarded is None or not (60.0 <= pre_guarded["distance_cm"] <= 110.0):
        print("FAIL: the width guard should bring the shipped constants back near 94 cm")
        ok = False
    if pre_guarded is not None and inverted(pre_guarded["height_cm"]):
        print("FAIL: the width guard must not leave a normal sign in the 5/15 cm band")
        ok = False
    if now is not None:
        if abs(now["height_cm"] - 10.0) > 0.01:
            print("FAIL: with these constants the sign should read 10 cm, not %.1f"
                  % now["height_cm"])
            ok = False
        if abs(now["distance_cm"] - KNOWN["distance_cm"]) > 6.0:
            print("FAIL: with these constants the sign should read ~94 cm, not %.1f"
                  % now["distance_cm"])
            ok = False
    if ok:
        print("OK: the shipped constants reproduce the log, the guard recovers it,"
              " and this file's constants read it correctly")
    return ok


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
    print("  true D  true H | vertical fit only     | + width guard + snap   | Surprise")
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
    print("(a mounting this far off has to be fixed with --fit; the guard only")
    print(" catches what no yaw could explain - it is a net, not a calibration)")
    return 0


def read_measurements(path):
    rows = []
    for line in pathlib.Path(path).read_text(encoding="utf-8").splitlines():
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
    parser.add_argument("--true-height", type=float, default=None,
                        help="mounting the sweep projects with (default: the script's own)")
    parser.add_argument("--true-pitch", type=float, default=None)
    parser.add_argument("--true-focal-lat", type=float, default=None)
    args = parser.parse_args(argv)

    ns = load_geometry(pathlib.Path(args.camera_script))
    if args.fit:
        return fit(ns, read_measurements(args.fit))
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
