from __future__ import annotations

import asyncio
import json
import math
from typing import Optional

import websockets
from websockets.server import WebSocketServerProtocol

from .hal import Hal, TelemetryState


class TelemetryServer:
  """WebSocket server that receives telemetry and can send control commands."""

  def __init__(self, host: str = "0.0.0.0", port: int = 8765) -> None:
    self.host = host
    self.port = port
    self.state = TelemetryState()
    self.hal = Hal(self.state)
    self._clients: set = set()
    self._driver: Optional[WebSocketServerProtocol] = None
    self._lock = asyncio.Lock()
    self._connected = asyncio.Event()
    # Optional synchronous hook, called for every telemetry frame. The AGSG
    # controller uses it so encoder deltas are forwarded as they arrive instead
    # of being sampled at the sensor tick.
    self.on_telemetry = None

  async def _handle_message(self, message: str, sender=None) -> None:
    # Observers may watch, but only the driver's world may move the car: two
    # browsers sending telemetry at once would fight over the same firmware.
    from_driver = sender is None or sender is self._driver
    try:
      data = json.loads(message)
    except json.JSONDecodeError:
      return
    msg_type = data.get("type")
    # Starting a round makes you the driver: whoever presses "New track" takes
    # over from the previous tab instead of silently driving a puppet world.
    if msg_type == "race" and data.get("action") == "reset" and sender is not None:
      if sender is not self._driver:
        self._driver = sender
        print("[ws] driver taken over", flush=True)
        self.queue_log("Controlo assumido por um novo cliente.")
    if not from_driver:
      return
    if msg_type in {"encoder", "ultrasonic", "sign", "signs", "heading"}:
      try:
        self.state.update_from_message(data)
      except Exception as exc:  # noqa: BLE001 - a bad frame must not kill the socket
        self.queue_log("telemetry frame rejected: %s" % exc)
    # Every frame reaches the hook: the AGSG controller also needs the control
    # channel (e.g. a race restart request), not only sensor data.
    hook = self.on_telemetry
    if hook is not None:
      try:
        hook(data)
      except Exception:
        # A controller must never take the socket down.
        pass

  def queue_log(self, message: str) -> None:
    """Fire-and-forget log, safe to call from a callback."""
    try:
      loop = asyncio.get_running_loop()
    except RuntimeError:
      return
    loop.create_task(self.send_log(message))

  async def _client_loop(self, websocket: WebSocketServerProtocol) -> None:
    address = websocket.remote_address
    self._clients.add(websocket)
    if self._driver is None:
      self._driver = websocket
      print(f"[ws] driver = {address}", flush=True)
    else:
      print(f"[ws] observer {address} (driver is {self._driver.remote_address})", flush=True)
    self._connected.set()
    try:
      async for message in websocket:
        await self._handle_message(message, websocket)
    except Exception as exc:  # noqa: BLE001
      print(f"[ws] handler error: {exc!r}", flush=True)
    finally:
      self._clients.discard(websocket)
      print(f"[ws] client gone {address}", flush=True)
      if self._driver is websocket:
        # Promote the oldest observer so a reloaded tab keeps working.
        self._driver = next(iter(self._clients), None)
        if self._driver is not None:
          print(f"[ws] driver = {self._driver.remote_address}", flush=True)
      if not self._clients:
        self._connected.clear()
        self.state.reset()

  async def _broadcast(self, payload: dict) -> None:
    """Send to every connected client; control and logs are for all of them."""
    if not self._clients:
      return
    data = json.dumps(payload)
    for ws in list(self._clients):
      try:
        await ws.send(data)
      except Exception:
        self._clients.discard(ws)

  async def send_control(
    self,
    velocity: float = 0.0,
    steering_radius: Optional[float] = None,
    odom: Optional[dict] = None,
    debug: Optional[dict] = None
  ) -> None:
    if not self._clients:
      return
    radius = steering_radius

    pose = odom if odom is not None else {
      "x": self.state.odom.x,
      "y": self.state.odom.y,
      "headingDeg": self.state.odom.headingDeg,
    }
    payload = {
      "type": "control",
      # Simulated milliseconds when this frame left the controller. The firmware
      # pose in it was computed one control period earlier, so a consumer that
      # wants to compare odometry against the truth must pair it with the world
      # pose at ~this instant, not at the moment the message happens to arrive.
      "simClockMs": getattr(self.state, "sim_clock_ms", None),
      "speed": velocity,
      # Send steering radius for both snake/camel consumers; omega is no longer transmitted.
      "steeringRadius": radius,
      "steering_radius": radius,
      "odom": pose,
    }
    if debug is not None:
      payload["debug"] = debug
    await self._broadcast(payload)

  async def send_sync(self, payload: dict) -> None:
    """Send a pacing message to the UI (lock-step advance, mode switches)."""
    await self._broadcast(payload)

  async def send_log(self, message: str) -> None:
    """Send a log line to every connected UI, if any."""
    if not self._clients:
      return
    await self._broadcast({"type": "log", "message": str(message)})

  async def serve_forever(self) -> None:
    async with websockets.serve(self._client_loop, self.host, self.port):
      await asyncio.Future()

  async def wait_connected(self) -> None:
    await self._connected.wait()

  def is_connected(self) -> bool:
    return self._connected.is_set()


