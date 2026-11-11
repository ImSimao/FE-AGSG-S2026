"""Headless race runner.

Drives the browser UI (which owns the physics) and the Python controller (which
owns the firmware) without a human at the keyboard, then prints the score as
JSON. Use it to run the same track N times after a firmware change.

    .venv/bin/python -m simulator.race_run --runs 3 --seconds 200
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import pathlib
import sys
import time

from playwright.async_api import async_playwright

DEFAULT_CHROME = os.environ.get(
  "CHROME_PATH",
  str(pathlib.Path.home() / ".cache/ms-playwright/chromium-1234/chrome-linux64/chrome"),
)


async def run_once(page, url: str, seconds: float, screenshot: pathlib.Path | None, new_track: bool) -> dict:
  await page.goto(url)
  await page.wait_for_function(
    "window.__race && window.__race.getState().phase !== 'idle'", timeout=20000
  )
  await page.evaluate("(newTrack) => window.__race.start(newTrack)", new_track)
  await page.wait_for_function(
    "window.__race.getState().phase === 'running'", timeout=30000
  )

  deadline = time.monotonic() + seconds
  while time.monotonic() < deadline:
    state = await page.evaluate("() => window.__race.getState().phase")
    if state == "finished":
      break
    await asyncio.sleep(0.5)

  result = await page.evaluate("() => window.__race.getResult()")
  result["phase"] = await page.evaluate("() => window.__race.getState().phase")
  result["wall_clock_s"] = round(time.monotonic() - (deadline - seconds), 1)
  if screenshot is not None:
    await page.screenshot(path=str(screenshot))
    result["screenshot"] = str(screenshot)
  return result


async def main() -> int:
  parser = argparse.ArgumentParser(description="Headless WRO FE simulated round")
  parser.add_argument("--url", default="http://127.0.0.1:8781/")
  parser.add_argument("--runs", type=int, default=1)
  parser.add_argument("--seconds", type=float, default=180.0, help="wall-clock cap per run")
  parser.add_argument("--out", default="race-results.json")
  parser.add_argument("--shot", default="", help="screenshot prefix (optional)")
  parser.add_argument("--same-track", action="store_true", help="reuse the track between runs")
  parser.add_argument("--chrome", default=DEFAULT_CHROME)
  args = parser.parse_args()

  results = []
  async with async_playwright() as p:
    browser = await p.chromium.launch(
      executable_path=args.chrome,
      args=["--no-sandbox", "--disable-dev-shm-usage", "--use-gl=swiftshader"],
    )
    context = await browser.new_context(viewport={"width": 1400, "height": 1000})
    page = await context.new_page()
    page.on("console", lambda msg: print(f"[console:{msg.type}] {msg.text}", file=sys.stderr))
    page.on("pageerror", lambda exc: print(f"[pageerror] {exc}", file=sys.stderr))

    for i in range(args.runs):
      shot = pathlib.Path(f"{args.shot}-{i + 1}.png") if args.shot else None
      print(f"=== run {i + 1}/{args.runs} ===", file=sys.stderr)
      result = await run_once(page, args.url, args.seconds, shot, (i == 0) and not args.same_track)
      result["run"] = i + 1
      results.append(result)
      print(json.dumps(result, ensure_ascii=False), file=sys.stderr)

    await browser.close()

  pathlib.Path(args.out).write_text(json.dumps(results, indent=2, ensure_ascii=False))
  print(json.dumps(results, indent=2, ensure_ascii=False))
  return 0


if __name__ == "__main__":
  raise SystemExit(asyncio.run(main()))
