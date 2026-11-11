#!/usr/bin/env python3
"""Margin report for a recorded WRO Future Engineers round.

Reads the JSON the simulator exports and answers one question per section: how
much room did the car actually have? Every wall and every traffic sign is scored
by the clearance between it and the car's body box, so a line that looks tidy on
the plot but grazes a sign at 1 cm shows up as 1 cm.

    .venv/bin/python tools/margin_report.py prova-....json
    .venv/bin/python tools/margin_report.py exports/*.json --top 5

Geometry mirrors ui/lib/config.js and ui/lib/geometry.js: the pose in the export
is the odometry origin (under the rear axle) and the body box is built around
`ARM` metres ahead of it.

Nothing here is scored by the rules except a contact: passing a sign on the
"wrong" side costs nothing in this simulator, and the report says so instead of
inventing a penalty.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass

# --- world, from ui/lib/config.js ------------------------------------------
ARENA = 3.0          # m, field is 3 x 3
SQUARE = 1.0         # m, inner square
ROBOT_L = 0.127      # m
ROBOT_W = 0.061      # m
ARM = 0.038          # m, rear axle -> body centre
SIGN = 0.05          # m
BAY_GAP = 0.1905     # m, bay length along the wall = 1.5 x 127 mm chassis
BAY_DEPTH = 0.20     # m, always 20 cm out from the wall
BAY_WALL_T = 0.02    # m, magenta elements 200 x 20 x 100 mm
WALL_T = 0.01

CX = ARENA / 2
HALF = SQUARE / 2


def walls() -> list[tuple[str, tuple[tuple[float, float], tuple[float, float]]]]:
    """The eight solid lines the car can touch: outer boundary + inner square."""
    lo, hi = 0.0, ARENA
    a0, a1 = CX - HALF, CX + HALF
    return [
        ("outer south", ((lo, lo), (hi, lo))),
        ("outer north", ((lo, hi), (hi, hi))),
        ("outer west", ((lo, lo), (lo, hi))),
        ("outer east", ((hi, lo), (hi, hi))),
        ("inner south", ((a0, a0), (a1, a0))),
        ("inner north", ((a0, a1), (a1, a1))),
        ("inner west", ((a0, a0), (a0, a1))),
        ("inner east", ((a1, a0), (a1, a1))),
    ]


def sign_edges(x: float, y: float):
    h = SIGN / 2
    return [
        ((x - h, y - h), (x + h, y - h)),
        ((x + h, y - h), (x + h, y + h)),
        ((x + h, y + h), (x - h, y + h)),
        ((x - h, y + h), (x - h, y - h)),
    ]


def box(pose: dict) -> list[tuple[float, float]]:
    rad = math.radians(pose["heading_deg"])
    cos, sin = math.cos(rad), math.sin(rad)
    cx = pose["x_cm"] / 100 + cos * ARM
    cy = pose["y_cm"] / 100 + sin * ARM
    hl, hw = ROBOT_L / 2, ROBOT_W / 2
    return [
        (cx + cos * hl - sin * hw, cy + sin * hl + cos * hw),
        (cx + cos * hl + sin * hw, cy + sin * hl - cos * hw),
        (cx - cos * hl + sin * hw, cy - sin * hl - cos * hw),
        (cx - cos * hl - sin * hw, cy - sin * hl + cos * hw),
    ]


def seg_point(seg, p) -> float:
    (x1, y1), (x2, y2) = seg
    px, py = p
    dx, dy = x2 - x1, y2 - y1
    den = dx * dx + dy * dy
    t = 0.0 if den == 0 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / den))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def seg_seg(a, b) -> float:
    """Distance between two segments (0 if they cross)."""
    d = min(
        min(seg_point(a, p) for p in b),
        min(seg_point(b, p) for p in a),
    )
    # crossing test: proper intersection => the box is through the wall
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    o1, o2 = orient(a[0], a[1], b[0]), orient(a[0], a[1], b[1])
    o3, o4 = orient(b[0], b[1], a[0]), orient(b[0], b[1], a[1])
    if (o1 * o2 < 0) and (o3 * o4 < 0):
        return 0.0
    return d


def box_edges(corners):
    return [(corners[i], corners[(i + 1) % 4]) for i in range(4)]


def clearance_to_segments(corners, segments) -> float:
    edges = box_edges(corners)
    return min(seg_seg(e, s) for e in edges for s in segments)


@dataclass
class Hit:
    what: str
    gap_cm: float
    t_ms: int
    x_cm: float
    y_cm: float


def lane_of(x_cm: float, y_cm: float) -> str:
    x, y = x_cm / 100, y_cm / 100
    if CX - HALF < x < CX + HALF and y <= CX - HALF:
        return "bottom"
    if CX - HALF < x < CX + HALF and y >= CX + HALF:
        return "top"
    if CX - HALF < y < CX + HALF and x >= CX + HALF:
        return "right"
    if CX - HALF < y < CX + HALF and x <= CX - HALF:
        return "left"
    return "corner"


def analyse(path: str) -> dict:
    data = json.load(open(path))
    traj = data["trajectory"]
    res = data["result"]
    sim = data.get("sim", {})

    named = walls()
    # bay walls (magenta), drawn at the two sides of the parking gap
    bx1 = CX + HALF - BAY_WALL_T
    bx0 = bx1 - BAY_GAP
    named += [
        ("bay inner", ((bx0, 0.0), (bx0, BAY_DEPTH))),
        ("bay outer", ((bx1, 0.0), (bx1, BAY_DEPTH))),
    ]
    solid = [seg for _, seg in named]

    first_move = next((s["t_ms"] for s in traj if abs(s["speed_cm_s"]) > 1e-9), None)
    if first_move is None:
        raise SystemExit(f"{path}: nothing moved in this round")

    laps = [first_move] + [first_move + ms for ms in res["lapTimesMs"]]

    per_lane: dict[str, Hit] = {}
    contacts: list[Hit] = []
    for s in traj:
        if s["t_ms"] < first_move:
            continue
        corners = box(s)
        gap = clearance_to_segments(corners, solid)
        lane = lane_of(s["x_cm"], s["y_cm"])
        if lane not in per_lane or gap < per_lane[lane].gap_cm:
            per_lane[lane] = Hit(lane, gap * 100, s["t_ms"], s["x_cm"], s["y_cm"])
        if gap <= 0.001:
            if not contacts or s["t_ms"] - contacts[-1].t_ms > 200:
                contacts.append(Hit(lane, gap * 100, s["t_ms"], s["x_cm"], s["y_cm"]))

    # sign clearance, and the closest approach to any sign
    worst_sign = None
    for sign in data.get("fieldSigns", []):
        segs = sign_edges(sign["x_cm"] / 100, sign["y_cm"] / 100)
        for s in traj:
            if s["t_ms"] < first_move:
                continue
            gap = clearance_to_segments(box(s), segs)
            if worst_sign is None or gap < worst_sign.gap_cm / 100:
                worst_sign = Hit(
                    f"sinal {sign['colour']} {sign['lane']} ({sign['x_cm']:.0f},{sign['y_cm']:.0f})",
                    gap * 100, s["t_ms"], s["x_cm"], s["y_cm"],
                )

    # per-lap summary
    lap_rows = []
    for i in range(len(laps) - 1):
        w = [s for s in traj if laps[i] <= s["t_ms"] <= laps[i + 1]]
        if not w:
            continue
        v = [abs(s["speed_cm_s"]) for s in w]
        rev = sum(1 for s in w if s["speed_cm_s"] < -1) * (w[1]["t_ms"] - w[0]["t_ms"]) / 1000
        path_len = sum(
            math.hypot(b["x_cm"] - a["x_cm"], b["y_cm"] - a["y_cm"])
            for a, b in zip(w, w[1:])
        ) / 100
        lap_rows.append({
            "lap": i + 1,
            "time_s": (laps[i + 1] - laps[i]) / 1000,
            "path_m": path_len,
            "v_mean": sum(v) / len(v),
            "slow_pct": 100 * sum(1 for x in v if x < 15) / len(v),
            "reverse_s": rev,
        })

    passages = data.get("signPassages", [])
    return {
        "path": path,
        "seed": sim.get("seed"),
        "direction": sim.get("direction"),
        "score": res.get("score"),
        "elapsed_s": res["elapsedMs"] / 1000,
        "laps": lap_rows,
        "lanes": per_lane,
        "contacts": contacts,
        "worst_sign": worst_sign,
        "passages": passages,
        "wrong_side": [p for p in passages if not p.get("on_intended_side")],
        "wall_moved": res.get("wallMoved"),
        "parking": res.get("parking"),
    }


def report(r: dict, top: int = 3) -> None:
    print(f"{r['path']}")
    print(f"  seed {r['seed']} · {r['direction']} · {r['elapsed_s']:.1f} s · "
          f"{r['score']} pts · estacionamento {r['parking'] or '—'}"
          + (" · PAREDE MOVIDA" if r["wall_moved"] else ""))
    print()
    print("  volta  tempo   caminho  v.média  %<15cm/s  marcha-atrás")
    for l in r["laps"]:
        print(f"   {l['lap']}   {l['time_s']:6.1f}s {l['path_m']:6.2f} m "
              f"{l['v_mean']:6.1f} {l['slow_pct']:8.1f}% {l['reverse_s']:10.1f}s")

    print("\n  folga mínima às paredes, por secção")
    for lane in ("bottom", "left", "top", "right", "corner"):
        h = r["lanes"].get(lane)
        if not h:
            continue
        flag = "  CONTACTO" if h.gap_cm <= 0.5 else ("  <5 mm" if h.gap_cm < 0.5 else "")
        print(f"   {lane:<7} {h.gap_cm:6.1f} cm  em t={h.t_ms/1000:6.1f}s "
              f"({h.x_cm:5.1f},{h.y_cm:5.1f}){flag}")

    if r["worst_sign"]:
        w = r["worst_sign"]
        print(f"\n   sinal mais próximo: {w.gap_cm:.1f} cm  ({w.what}) em t={w.t_ms/1000:.1f}s")

    if r["contacts"]:
        print(f"\n  contactos com parede: {len(r['contacts'])}")
        for c in r["contacts"][:top]:
            print(f"   t={c.t_ms/1000:6.1f}s {c.what} ({c.x_cm:.1f},{c.y_cm:.1f})")

    print(f"\n  passagens de sinais: {len(r['passages'])}"
          f" · fora da linha: {len(r['wrong_side'])}")
    for p in r["wrong_side"]:
        dy = p["car_y_cm"] - p["sign_y_cm"]
        dx = p["car_x_cm"] - p["sign_x_cm"]
        print(f"   t={p['t_ms']/1000:6.1f}s {p['colour']:<5} {p['lane']:<6} "
              f"sinal ({p['sign_x_cm']:.0f},{p['sign_y_cm']:.0f}) "
              f"carro ({p['car_x_cm']:.0f},{p['car_y_cm']:.0f}) "
              f"-> {p['side']} (linha {p['intended_side']}), desvio {math.hypot(dx, dy):.1f} cm")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("exports", nargs="+")
    ap.add_argument("--top", type=int, default=3)
    args = ap.parse_args()
    for i, path in enumerate(args.exports):
        if i:
            print("\n" + "-" * 60 + "\n")
        report(analyse(path), args.top)


if __name__ == "__main__":
    main()
