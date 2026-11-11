#!/usr/bin/env bash
set -euo pipefail

# This script lives in src/simulator/. It runs from src/ so that
# `python -m simulator` resolves the simulator package, while the firmware is
# the sibling src/robot/.
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(dirname "$SIM_DIR")"
cd "$SRC_DIR"

PYTHON_BIN="${PYTHON:-python3}"
UI_PORT="${UI_PORT:-8781}"
SIM_HOST="${SIM_HOST:-0.0.0.0}"
SIM_PORT="${SIM_PORT:-8780}"

if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
  echo "Python interpreter not found: $PYTHON_BIN" >&2
  exit 1
fi

# Activate repo-local virtualenv if present.
if [ -d "$SRC_DIR/.venv" ]; then
  # shellcheck disable=SC1091
  source "$SRC_DIR/.venv/bin/activate"
fi

# Ensure simulator dependency is available.
if ! "$PYTHON_BIN" - <<'PY' >/dev/null 2>&1; then
try:
    import websockets  # type: ignore
except Exception:
    raise SystemExit(1)
PY
  echo "[setup] Installing simulator dependencies..."
  "$PYTHON_BIN" -m pip install -r "$SIM_DIR/requirements.txt"
fi

cleanup() {
  [[ -n "${UI_PID:-}" ]] && kill "$UI_PID" 2>/dev/null || true
  [[ -n "${SIM_PID:-}" ]] && kill "$SIM_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[ui] Serving ./simulator/ui at http://localhost:${UI_PORT}"
"$PYTHON_BIN" -m http.server "$UI_PORT" --directory "$SIM_DIR/ui" &
UI_PID=$!

echo "[simulator] Starting WebSocket server at ws://${SIM_HOST}:${SIM_PORT}"
"$PYTHON_BIN" -m simulator.main --host "$SIM_HOST" --port "$SIM_PORT" &
SIM_PID=$!

echo "[start] Press Ctrl+C to stop both processes"
# Wait for either process to exit (Ctrl+C will terminate both).
wait -n "$UI_PID" "$SIM_PID"
