#!/usr/bin/env python3
"""Print the console snippet that reproduces a run's traffic signs in the UI.

The UI exposes window.__race.field.set([{x_cm, y_cm, state}]); state is 1 for
green and 2 for red, and anything not listed is cleared. The exports the sweep
writes carry fieldSigns with the colour as a name, so this converts one into the
other and prints a line you can paste in the browser console of the UI page.

    python3 tools/field_from_export.py /tmp/all_cw/bottom_rg_a12.json
    python3 tools/field_from_export.py --all /tmp/all_cw /tmp/all_ccw   # only the failures

Paste it while the page is armed (the round is started with "Iniciar"), not
before: field.set with the world still idle leaves nothing to place the signs on.
"""

from __future__ import annotations

import argparse
import glob
import json
import pathlib
import sys
import urllib.parse

STATE = {"green": 1, "red": 2}


def snippet(export: pathlib.Path, with_url: bool = False) -> str:
    data = json.loads(export.read_text())
    signs = data.get("fieldSigns") or []
    if not signs:
        raise SystemExit(f"{export}: no fieldSigns in this export")
    body = ", ".join(
        "{x_cm: %g, y_cm: %g, state: %d}" % (s["x_cm"], s["y_cm"], STATE[s["colour"]])
        for s in signs
    )
    sim = data.get("sim") or {}
    head = "# %s (%s %s, seed %s, %d sinais)" % (
        export.name, sim.get("direction"), sim.get("startMode"),
        sim.get("seed"), len(signs),
    )
    line = "window.__race.field.set([%s]);" % body
    if with_url:
        # The same layout as a URL, applied on every start, so the page can be
        # reloaded without pasting anything.
        compact = ";".join(
            "%g,%g,%d" % (s["x_cm"], s["y_cm"], STATE[s["colour"]]) for s in signs
        )
        line = (
            "http://<host>:8781/?dir=%s&start=%s&seed=%s&signs=%s"
            % (sim.get("direction"), sim.get("startMode", "wall"),
               sim.get("seed"), urllib.parse.quote(compact, safe=",;"))
        )
    return head + "\n" + line


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("export", nargs="*", help="export .json from a round")
    ap.add_argument("--url", action="store_true",
                    help="print a UI URL instead of a console line")
    ap.add_argument("--all", nargs="*", metavar="DIR",
                    help="directories of exports: prints only the runs below 62 points")
    args = ap.parse_args()
    paths = [pathlib.Path(p) for p in args.export]
    if args.all:
        for d in args.all:
            for p in sorted(glob.glob(str(pathlib.Path(d) / "*.json"))):
                if p.endswith("summary.json"):
                    continue
                data = json.loads(pathlib.Path(p).read_text())
                if (data.get("result") or {}).get("score", 62) < 62:
                    paths.append(pathlib.Path(p))
    if not paths:
        ap.error("pass an export or --all DIR")
    for p in paths:
        print(snippet(p, with_url=args.url))
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
