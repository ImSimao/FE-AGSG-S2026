#!/usr/bin/env python3
"""Sweep every traffic-sign hypothesis (clockwise) and score the firmware.

Each case injects an exact sign layout into the world (`window.__race.field.set`),
starts a round and exports the JSON. Results are appended to a summary file so the
sweep can run in chunks and be resumed.

    python3 tools/sign_sweep.py --out /tmp/sweep --speed 4
    python3 tools/sign_sweep.py --out /tmp/sweep --list          # what will run
    python3 tools/sign_sweep.py --out /tmp/sweep --only left_rg_ab

Sign slots, per lane (the field puts them at 100/150/200 cm along the lane, on
one of two rows):

    bottom   row A: y=60                     x=100,150,200
    left     row A: x=60   row B: x=40       y=100,150,200
    top      row A: y=260  row B: y=240      x=100,150,200
    right    row A: x=240  row B: x=260      y=100,150,200

`A` is the row nearest the outer wall for left/right, nearest the top wall for
top; for bottom there is only one row.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import pathlib
import socket
import subprocess
import sys
import time

CHROME = str(pathlib.Path.home() / ".cache/ms-playwright/chromium-1234/chrome-linux64/chrome")

SLOTS = {
    "bottom": {"A": [(100, 60), (150, 60), (200, 60)]},
    "left": {"A": [(60, 100), (60, 150), (60, 200)],
             "B": [(40, 100), (40, 150), (40, 200)]},
    "top": {"A": [(100, 260), (150, 260), (200, 260)],
            "B": [(100, 240), (150, 240), (200, 240)]},
    "right": {"A": [(240, 100), (240, 150), (240, 200)],
              "B": [(260, 100), (260, 150), (260, 200)]},
}

G, R = 1, 2

# (slot index 0/1/2, row, colour). Covered cases, per lane:
#   same colour on one row, first+last slot      -> straight line
#   opposite colours on one row                  -> switch, same row
#   opposite colours on opposite rows            -> switch across rows (2026)
#   same colour on opposite rows                 -> rows must not matter
#   adjacent slots (100,150)                     -> early switch
#   single sign                                  -> one line only
PATTERNS = {
    "gg_a13":  [(0, "A", G), (2, "A", G)],
    "rr_a13":  [(0, "A", R), (2, "A", R)],
    "rg_a13":  [(0, "A", R), (2, "A", G)],
    "gr_a13":  [(0, "A", G), (2, "A", R)],
    "rg_ab13": [(0, "A", R), (2, "B", G)],
    "gr_ab13": [(0, "A", G), (2, "B", R)],
    "gg_ab13": [(0, "A", G), (2, "B", G)],
    "rr_ab13": [(0, "A", R), (2, "B", R)],
    # A single sign, at the slot the field puts in the middle of the lane. The
    # pair at slots 0 and 1 that this dict used to have was not a layout the
    # field can produce: two signs in one lane are always at the two ends, at
    # least 50 cm apart, and the middle slot only ever appears alone
    # (ui/lib/signsControl.js: randomizeSigns picks the pair from the non-centre
    # slots and enforces minSeparation = 0.5).
    "g_c1":    [(1, "A", G)],
    "r_c1":    [(1, "A", R)],
    "g_a1":    [(0, "A", G)],
    "r_a1":    [(0, "A", R)],
}

# One row per lane; a lane never carries a middle sign next to an end one, and
# the two signs of a lane sit at the ends of its slot row.
MIN_SEPARATION_CM = 50.0


def check_layout(spec):
    """Reject sign layouts the field cannot produce.

    Measured the hard way: eight cases of the first full sweep used a sign at
    100 and one at 150 on the same lane, which the field never does, and their
    wrong-side failures were read as firmware defects for a while.
    """
    from collections import defaultdict

    per_lane = defaultdict(list)
    for sign in spec:
        lane = ("bottom" if sign["y_cm"] == 60 else
                "top" if sign["y_cm"] in (240, 260) else
                "left" if sign["x_cm"] in (40, 60) else "right")
        per_lane[lane].append((sign["x_cm"], sign["y_cm"]))
    for lane, points in per_lane.items():
        if len(points) > 2:
            return f"{lane}: {len(points)} signs in one lane (at most 2)"
        centre = {"bottom": 150, "top": 150}.get(lane, 150)
        along = 0 if lane in ("bottom", "top") else 1
        coords = [p[along] for p in points]
        if len(points) == 2:
            if centre in coords:
                return f"{lane}: a middle sign next to an end one at {points}"
            if abs(coords[0] - coords[1]) < MIN_SEPARATION_CM:
                return f"{lane}: signs {abs(coords[0] - coords[1])} cm apart, below {MIN_SEPARATION_CM}"
    return None

BASE_LANE = "rr_a13"  # a lane driven on one line, used as the backdrop


def build_cases():
    """One lane varied at a time, plus every lane on the same extreme."""
    cases = []
    for lane in ("bottom", "left", "top", "right"):
        for name, pattern in PATTERNS.items():
            spec = []
            for other in ("bottom", "left", "top", "right"):
                slots = SLOTS[other]
                rows = list(slots)
                chosen = pattern if other == lane else PATTERNS[BASE_LANE]
                for slot, row, colour in chosen:
                    if row not in slots:  # bottom has a single row
                        row = rows[0]
                    x, y = slots[row][slot]
                    spec.append({"x_cm": x, "y_cm": y, "state": colour})
            cases.append((f"{lane}_{name}", spec))

    for name in ("rr_a13", "gg_a13", "rg_a13", "rg_ab13", "gr_a13"):
        spec = []
        for lane in ("bottom", "left", "top", "right"):
            slots = SLOTS[lane]
            for slot, row, colour in PATTERNS[name]:
                if row not in slots:
                    row = list(slots)[0]
                x, y = slots[row][slot]
                spec.append({"x_cm": x, "y_cm": y, "state": colour})
        cases.append((f"all_{name}", spec))

    impossible = [f"{name}: {why}" for name, spec in cases if (why := check_layout(spec))]
    if impossible:
        raise SystemExit(
            "these layouts are not layouts the field can produce:\n  "
            + "\n  ".join(impossible)
        )
    return cases


async def run_case(browser, case, out_dir, url_base, cap_ms):
    """One round in a fresh tab of a browser the worker keeps alive.

    Launching Chromium per case cost 2-3 s of the ~35 s a 4x round takes, and it
    was the only setup between one case ending and the next starting. The tab is
    still new: a round must not inherit the previous world, the previous firmware
    process or the previous stream timers.
    """
    name, spec = case
    path = out_dir / f"{name}.json"
    if path.exists():
        return json.loads(path.read_text()).get("summary")

    ctx = await browser.new_context(accept_downloads=True)
    try:
        page = await ctx.new_page()
        await page.goto(url_base)
        await page.wait_for_function(
            "window.__race && window.__race.getState().phase !== 'idle'", timeout=30000
        )
        await page.evaluate("window.__race.start(false)")
        placed = await page.evaluate(
            "async (spec) => window.__race.field.set(spec)", spec
        )
        # The round is armed, not running: the clock starts on the first
        # movement, and polling before that returned phase="armed" and saved an
        # empty export.
        await page.wait_for_function(
            "window.__race.getState().phase === 'running'", timeout=40000
        )
        t0 = time.monotonic()
        while time.monotonic() - t0 < 400:
            st = await page.evaluate(
                "() => {const s = window.__race.getState();"
                " return {phase: s.phase, el: s.elapsedMs, sec: s.sectionsDone,"
                " laps: s.laps, score: s.score};}"
            )
            if st["phase"] != "running" or (st["el"] or 0) >= cap_ms:
                break
            await asyncio.sleep(0.15)
        async with page.expect_download() as dl:
            await page.click("#exportJsonBtn")
        download = await dl.value
        await download.save_as(str(path))
        data = json.loads(path.read_text())
        res = data["result"]
        passages = data.get("signPassages", [])
        # Drop the boot transient: the firmware's odometry origin is (0,0) until
        # its first set_relative_odom, so the first samples measure the bay
        # against the map corner and read ~200 cm.
        odom = [o for o in data.get("odometry", [])
                if o["t_ms"] >= 2000 and (abs(o["fw_x_cm"]) + abs(o["fw_y_cm"])) > 1]
        summary = {
            "case": name,
            "placed": placed,
            "score": res["score"],
            "laps": res["laps"],
            "sections": res["sectionsDone"],
            "seconds": round((res.get("elapsedMs") or 0) / 1000, 1),
            "parking": res.get("parking"),
            "wallMoved": res.get("wallMoved"),
            "signsMoved": res.get("signsMoved"),
            "wrongSide": res.get("signWrongSide"),
            "wrongSideCount": sum(1 for p in passages if p.get("on_intended_side") is False),
            "passages": len(passages),
            "odomErrMaxCm": max((o["err_cm"] for o in odom), default=None),
            "odomErrMedCm": round(sorted(o["err_cm"] for o in odom)[len(odom) // 2], 1) if odom else None,
            "odomErrEndCm": odom[-1]["err_cm"] if odom else None,
            "headingErrMaxDeg": max((abs(o["heading_err_deg"]) for o in odom), default=None),
        }
        return summary
    finally:
        # The browser belongs to the lane and is reused by the next case; only
        # this round's context dies here.
        await ctx.close()


_LOG_HANDLES = []


def wait_listening(port, timeout=30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with socket.socket() as sock:
            if sock.connect_ex(("127.0.0.1", port)) == 0:
                return True
        time.sleep(0.2)
    return False


def start_controllers(count, base_port, repo):
    """One controller per worker: a round owns the firmware process for its port."""
    procs = []
    global _LOG_HANDLES
    for i in range(count):
        port = base_port + i
        log = open(f"/tmp/sign_sweep_{port}.log", "wb")
        _LOG_HANDLES.append(log)   # keep the handle alive or the child's stdout goes EPIPE
        procs.append((port, subprocess.Popen(
            [sys.executable, "-m", "simulator.main", "--port", str(port)],
            cwd=str(repo), stdout=log, stderr=subprocess.STDOUT,
        )))
    for port, _ in procs:
        if not wait_listening(port):
            raise RuntimeError(f"controller on {port} did not start")
    return procs


async def main_async(args):
    from playwright.async_api import async_playwright

    out_dir = pathlib.Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    cases = build_cases()
    if args.only:
        wanted = set(args.only.split(","))
        cases = [c for c in cases if c[0] in wanted]
    if args.list:
        for name, spec in cases:
            print(name, len(spec), "signs")
        return
    ports = [args.ws_port]
    procs = []
    if args.workers > 1:
        repo = pathlib.Path(__file__).resolve().parent.parent
        started = start_controllers(args.workers, args.base_port, repo)
        ports = [port for port, _ in started]
        procs = [proc for _, proc in started]

    def url_for(port):
        return (
            f"http://127.0.0.1:{args.ui}/?dir={args.dir}&start=wall&seed={args.seed}"
            f"&speed={args.speed}&ws=ws://127.0.0.1:{port}"
        )

    rows = []
    printed = asyncio.Lock()
    pending: asyncio.Queue = asyncio.Queue()
    for case in cases:
        pending.put_nowait(case)

    async def lane(port, pw):
        """One worker: one controller, one browser, cases back to back.

        The browser is launched once per lane, so the hand-off between rounds is
        a new tab rather than a new Chromium; the next case leaves the queue the
        moment the previous round's export is written.
        """
        browser = await pw.chromium.launch(executable_path=CHROME, args=["--no-sandbox"])
        try:
            while True:
                try:
                    case = pending.get_nowait()
                except asyncio.QueueEmpty:
                    return
                started = time.monotonic()
                try:
                    row = await run_case(browser, case, out_dir, url_for(port), args.cap_ms)
                except Exception as exc:  # one bad case must not stop the sweep
                    print("%-16s ERRO %s" % (case[0], exc), flush=True)
                    continue
                wall = time.monotonic() - started
                if row is None:
                    continue
                row["wallSeconds"] = round(wall, 1)
                rows.append(row)
                async with printed:
                    print(
                        "%-16s score %2d  laps %d  sec %2d  %5.1fs  wall %4.1fs  park %-5s"
                        "  wallMoved %-5s  wrongSide %d/%d  odomErr %.1fcm  headErr %.0f deg"
                        % (
                            row["case"], row["score"], row["laps"], row["sections"],
                            row["seconds"], wall, row["parking"] or "-", row["wallMoved"],
                            row["wrongSideCount"], row["passages"],
                            row["odomErrMaxCm"] or 0, row["headingErrMaxDeg"] or 0,
                        ),
                        flush=True,
                    )
        finally:
            await browser.close()

    try:
        async with async_playwright() as pw:
            await asyncio.gather(*(lane(port, pw) for port in ports))
    finally:
        for proc in procs:
            proc.terminate()
    summary_path = out_dir / "summary.json"
    existing = []
    if summary_path.exists():
        existing = json.loads(summary_path.read_text())
        done = {r["case"] for r in rows}
        existing = [r for r in existing if r["case"] not in done]
    summary_path.write_text(json.dumps(existing + rows, indent=2))
    print(f"\n{len(rows)} casos -> {summary_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/sweep")
    ap.add_argument("--ui", default="8781")
    ap.add_argument("--ws-port", default="8792")
    ap.add_argument("--workers", type=int, default=1,
                    help="parallel rounds; each worker starts its own controller")
    ap.add_argument("--speed", default="4")
    ap.add_argument("--seed", default="4111204647")
    ap.add_argument("--cap-ms", type=float, default=150000)
    ap.add_argument("--only", default="")
    ap.add_argument("--dir", default="cw", choices=["cw", "ccw"],
                    help="which way round the field the round is driven")
    ap.add_argument("--base-port", type=int, default=8801,
                    help="first controller port when --workers > 1; two sweeps "
                         "running at once need different ranges")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()
    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()
