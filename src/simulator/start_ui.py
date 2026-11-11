#!/usr/bin/env python3
"""
Lightweight UI-only dev server that works on Linux and Windows.

Usage:
  python start_ui.py --port 8000 --host 127.0.0.1 --open
Environment:
  UI_PORT: optional default port override (falls back to 8000).
"""

from __future__ import annotations

import argparse
import http.server
import os
import pathlib
import socketserver
import sys
import webbrowser

ROOT = pathlib.Path(__file__).resolve().parent
UI_DIR = ROOT / "ui"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Serve the UI for local development.")
  default_port = int(os.environ.get("UI_PORT", "8000"))
  parser.add_argument("--host", default="127.0.0.1", help="Host to bind (default: 127.0.0.1)")
  parser.add_argument("--port", type=int, default=default_port, help="Port to bind (default: 8000 or UI_PORT)")
  parser.add_argument("--open", action="store_true", dest="open_browser", help="Open the browser after starting")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  handler_cls = lambda *h_args, **h_kwargs: http.server.SimpleHTTPRequestHandler(  # noqa: E731
    *h_args, directory=str(UI_DIR), **h_kwargs
  )

  try:
    with socketserver.ThreadingTCPServer((args.host, args.port), handler_cls) as httpd:
      url = f"http://{args.host}:{args.port}/"
      print(f"[ui] Serving {UI_DIR} at {url}")
      print("[ui] Press Ctrl+C to stop")
      if args.open_browser:
        try:
          webbrowser.open(url)
        except Exception as err:  # pragma: no cover - best effort
          print(f"[ui] Could not open browser: {err}", file=sys.stderr)
      httpd.serve_forever()
  except OSError as err:
    print(f"[ui] Failed to start server: {err}", file=sys.stderr)
    return 1
  except KeyboardInterrupt:
    print("\n[ui] Stopped")
    return 0
  return 0


if __name__ == "__main__":
  raise SystemExit(main())



