#!/usr/bin/env python3
"""Summarise a sign_sweep run: which hypotheses fail, and why.

    python3 tools/sweep_report.py /tmp/sweep
"""

from __future__ import annotations

import json
import pathlib
import sys
from collections import defaultdict


def load(out_dir: pathlib.Path):
    summary = json.loads((out_dir / "summary.json").read_text())
    rows = {}
    for row in summary:
        rows[row["case"]] = row
    return rows


def passages(out_dir: pathlib.Path, case: str):
    path = out_dir / f"{case}.json"
    if not path.exists():
        return [], []
    data = json.loads(path.read_text())
    return data.get("signPassages", []), data.get("fieldSigns", [])


def main():
    out_dir = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/sweep")
    rows = load(out_dir)
    by_group = defaultdict(list)
    for name, row in rows.items():
        lane, _, pattern = name.partition("_")
        by_group[lane].append((pattern, row))

    print(f"{len(rows)} cases\n")
    header = f"{'pattern':<12}" + "".join(f"{g:>14}" for g in ("bottom", "left", "top", "right"))
    print(header)
    patterns = sorted({p for g in by_group.values() for p, _ in g})
    for pattern in patterns:
        cells = []
        for lane in ("bottom", "left", "top", "right"):
            hit = next((r for p, r in by_group[lane] if p == pattern), None)
            if hit is None:
                cells.append(f"{'-':>14}")
            elif hit["score"] >= 62:
                cells.append(f"{'OK 62':>14}")
            else:
                cells.append(f"{hit['score']:>3} pts {hit['sections']:>2}s".rjust(14))
        print(f"{pattern:<12}" + "".join(cells))

    print("\nfailures, with the sign and the site:")
    for name, row in sorted(rows.items()):
        if row["score"] >= 62:
            continue
        ps, field = passages(out_dir, name)
        wrong = [p for p in ps if p.get("on_intended_side") is False]
        detail = ""
        if wrong:
            w = wrong[0]
            detail = "%s %s at (%s,%s) passed %s with the car at (%s,%s)" % (
                w["colour"], w.get("lane"), w["sign_x_cm"], w["sign_y_cm"],
                w["side"], w["car_x_cm"], w["car_y_cm"],
            )
        flags = []
        if row.get("wallMoved"):
            flags.append("wall moved")
        if row.get("signsMoved"):
            flags.append("signs moved")
        print(
            "  %-16s %2d pts %2d sec %5.1fs park=%-5s %s %s"
            % (name, row["score"], row["sections"], row["seconds"],
               row["parking"] or "-", " ".join(flags), detail)
        )


if __name__ == "__main__":
    main()
