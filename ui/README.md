# Robot Arena Visualizer (Web Serial, Frontend Only)

Chrome-only canvas viewer that reads `x,y,heading` over Web Serial and draws the robot in a configurable arena. No backend required.

## Features
- Web Serial CSV reader (`x,y,heading[,front,rear,left,right]*CS`, meters + degrees, XOR checksum).
- Canvas map with grid, optional background image, configurable origin and scale.
- Pan/zoom, live status log.
- Two modes: Telemetry (serial data) and Play/Test (W/A/S/D plus click/drag/right-click to place/aim).
- Built-in simulator to generate poses without hardware.
- WebSocket telemetry bridge for Python/MicroPython-friendly controllers.

## Prereqs
- Chrome/Edge with Web Serial (use `http://localhost` or HTTPS).
- Python for a quick static server (or any static host).

## Run

### Quick start (UI + simulator)
```bash
cd /home/dev/Desktop/wro-2026-simulation
./start.sh
# UI: http://localhost:8000 (served from ./ui)
# WS: ws://localhost:8765 (simulator controller)
```

### Serve only the web UI
```bash
cd /home/dev/Desktop/wro-2026-simulation
python -m http.server 8000 --directory ui
# open http://localhost:8000 in Chrome
```

Cross-platform helper (Linux/Windows):
```bash
cd /home/dev/Desktop/wro-2026-simulation
python start_ui.py --port 8000 --host 127.0.0.1 --open
```

### Simulation mode with Python controller (manual)
1) In a second terminal:
```bash
cd /home/dev/Desktop/wro-2026-simulation
python -m venv .venv
. .venv/bin/activate
pip install -r simulator/requirements.txt
python -m simulator.main --host 0.0.0.0 --port 8765
```
2) Load the web UI (Telemetry mode is the default). The browser auto-connects to `ws://<host>:8765` and streams telemetry; the Python controller replies with control commands.

### WebSocket protocol
- Transport: JSON over WebSocket, one message per frame (~10 Hz by default).
- Outbound telemetry (browser → Python):
```json
{
  "type": "telemetry",
  "mode": "telemetry|play",
  "pose": { "x": 1.0, "y": 0.5, "headingDeg": 45 },
  "distances": { "Front": 0.30, "Rear": 0.50, "Left": 0.25, "Right": 0.28 },
  "sign": { "lane": "bottom", "state": 1, "distance": 0.8, "angleDeg": -15 },
  "timestampMs": 1710000000000
}
```
  - `distances` always uses meters.
  - `sign` is omitted/null when no sign is in view; `state` 1=green, 2=red.
- Inbound control (Python → browser):
```json
{
  "type": "control",
  "velocity": 0.15,
  "steering_radius": 0.6
}
```
  - `velocity` in m/s; `steering_radius` in meters (positive = CCW). `omegaDegPerSec` is no longer used. Missing fields default to 0/straight. Commands older than 400 ms are ignored. The browser starts the simulator automatically when control commands arrive.

## Use
1) Click **Connect** and choose the serial device (Chrome will prompt).
2) CSV per line: `x,y,heading[,front,rear,left,right]*CS` (meters, degrees CCW from +x). `CS` is an XOR of the ASCII payload before `*` encoded as two hex chars. Example: `0.5,1.2,45,0.30,0.25,0.40,0.35*3F`. Lines without `*CS` are accepted for compatibility but checksummed lines are preferred.
3) Pick a mode: Telemetry (live serial) or Play/Test (W/A/S/D simulator plus click/drag to place, right-click to set heading).
4) Adjust arena size, origin, and pixels-per-meter in the controls.
5) Scroll to zoom, drag to pan. Upload a background image if desired.

## Notes
- Works only in Chrome/Edge; Safari/Firefox lack Web Serial.
- `file://` will not work with Web Serial; use localhost/HTTPS.
- If the grid disappears when zoomed out, increase pixels-per-meter.

