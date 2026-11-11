"""Calibrate the world<->firmware interface by running the real car in the sim.

The simulator has three knobs that cannot be derived from the firmware source
alone and that must match the hardware:

  * ``--imu-sign``   the BNO085 heading sign relative to the world's CCW axes
  * ``--servo-sign`` which way a positive servo angle steers
  * ``start``        where the car sits in the bay (the firmware anchors its
                     odometry from the sonar at boot)

This script starts the controller itself for every combination, drives a short
headless round and reports how far the car got. The best row is the pairing to
use for scored runs until the bench calibration on the real robot replaces it.

    .venv/bin/python -m simulator.calibrate --seconds 40
"""

from __future__ import annotations

import argparse
import asyncio
import itertools
import json
import os
import pathlib
import signal
import subprocess
import sys
import time

from playwright.async_api import async_playwright

DEFAULT_CHROME = os.environ.get(
  "CHROME_PATH",
  str(pathlib.Path.home() / ".cache/ms-playwright/chromium-1234/chrome-linux64/chrome"),
)
ROOT = pathlib.Path(__file__).resolve().parent.parent


async def start_controller(python: str, port: int, imu: int, servo: int, gain: float = 1.0) -> subprocess.Popen:
  proc = subprocess.Popen(
    [
      python, "-u", "-m", "simulator.main",
      "--port", str(port),
      "--imu-sign", str(imu),
      "--servo-sign", str(servo),
      "--servo-gain", str(gain),
    ],
    cwd=str(ROOT),
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL,
  )
  # Wait for the port to accept connections.
  import socket

  for _ in range(100):
    with socket.socket() as sock:
      sock.settimeout(0.2)
      if sock.connect_ex(("127.0.0.1", port)) == 0:
        return proc
    await asyncio.sleep(0.1)
  raise RuntimeError("controller did not start")


def stop_controller(proc: subprocess.Popen) -> None:
  if proc.poll() is None:
    proc.send_signal(signal.SIGTERM)
    try:
      proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
      proc.kill()


async def trial(page, url: str, seconds: float) -> dict:
  await page.goto(url)
  await page.wait_for_function(
    "window.__race && window.__race.getState().phase !== 'idle'", timeout=20000
  )
  await page.evaluate("() => window.__race.start(false)")
  await page.wait_for_function("window.__race.getState().phase === 'running'", timeout=30000)
  deadline = time.monotonic() + seconds
  while time.monotonic() < deadline:
    if await page.evaluate("() => window.__race.getState().phase") == "finished":
      break
    await asyncio.sleep(0.4)
  state = await page.evaluate(
    """() => {
      const s = window.__race.getState();
      const a = window.__race.appState;
      return {
        seconds: s.elapsedMs / 1000,
        sections: s.sectionsDone,
        laps: s.laps,
        score: s.score,
        wallMoved: s.wallMoved,
        wallHits: s.wallHits,
        direction: s.direction,
        ghostErrorCm: a.ghostErrorMeters == null ? null : a.ghostErrorMeters * 100,
        pose: a.latestPose,
      };
    }"""
  )
  state["phase"] = await page.evaluate("() => window.__race.getState().phase")
  return state


async def main() -> int:
  parser = argparse.ArgumentParser(description="Sweep the world<->firmware sign conventions")
  parser.add_argument("--url", default="http://127.0.0.1:8781/")
  parser.add_argument("--port", type=int, default=8780)
  parser.add_argument("--seconds", type=float, default=40.0)
  parser.add_argument("--starts", default="nose,centre,nose_in")
  parser.add_argument("--imus", default="-1,1")
  parser.add_argument("--servos", default="1,-1")
  parser.add_argument(
    "--gains",
    default="1.0",
    help="Servo angle scale: d_eff = gain * angle. Below 1.0 means the real "
    "servo reaches less lock than the firmware assumes (R = L/tan(d_eff)).",
  )
  parser.add_argument("--chrome", default=DEFAULT_CHROME)
  parser.add_argument("--seed", type=int, default=1234, help="Fixed track seed so every combo runs the same layout")
  parser.add_argument("--out", default="calibration.json")
  args = parser.parse_args()

  starts = [s.strip() for s in args.starts.split(",") if s.strip()]
  imus = [int(s) for s in args.imus.split(",") if s.strip()]
  servos = [int(s) for s in args.servos.split(",") if s.strip()]
  gains = [float(s) for s in args.gains.split(",") if s.strip()]
  python = sys.executable

  rows = []
  async with async_playwright() as p:
    browser = await p.chromium.launch(
      executable_path=args.chrome,
      args=["--no-sandbox", "--disable-dev-shm-usage", "--use-gl=swiftshader"],
    )
    page = await (await browser.new_context(viewport={"width": 1400, "height": 1000})).new_page()

    for imu, servo, start, gain in itertools.product(imus, servos, starts, gains):
      label = f"imu={imu:+d} servo={servo:+d} start={start} gain={gain}"
      print(f"--- {label}", file=sys.stderr)
      proc = await start_controller(python, args.port, imu, servo, gain)
      try:
        url = (
          f"{args.url}{'&' if '?' in args.url else '?'}start={start}"
          f"&ws=ws://127.0.0.1:{args.port}&dir=ccw&seed={args.seed}"
        )
        result = await trial(page, url, args.seconds)
      finally:
        stop_controller(proc)
      row = {"imu": imu, "servo": servo, "start": start, "gain": gain, **result}
      rows.append(row)
      print(json.dumps(row, ensure_ascii=False), file=sys.stderr)

    await browser.close()

  def rank(row):
    # Sections first, then fewer wall hits, then the odometry error (a run that
    # survives by driving aimlessly has a huge ghost error and must not win).
    ghost = row.get("ghostErrorCm")
    return (
      row.get("sections", 0),
      -(row.get("wallHits", 0)),
      -(ghost if ghost is not None else 1e9),
      row.get("seconds", 0),
    )

  rows.sort(key=rank, reverse=True)
  pathlib.Path(args.out).write_text(json.dumps(rows, indent=2, ensure_ascii=False))
  print("\nrank  imu servo start      secs  sect  laps  hits  wall  ghosts(cm)")
  for i, row in enumerate(rows[:8], 1):
    ghost = row.get("ghostErrorCm")
    print(
      f"{i:>4}  {row['imu']:>+3} {row['servo']:>+5} {row['start']:<9} "
      f"{row.get('seconds', 0):6.1f} {row.get('sections', 0):5} {row.get('laps', 0):5} "
      f"{row.get('wallHits', 0):5} {str(row.get('wallMoved')):<5} "
      f"{'' if ghost is None else round(ghost)}"
    )
  best = rows[0]
  print(
    f"\nbest: --imu-sign {best['imu']:+d} --servo-sign {best['servo']:+d} start={best['start']}"
  )
  return 0


if __name__ == "__main__":
  raise SystemExit(asyncio.run(main()))
