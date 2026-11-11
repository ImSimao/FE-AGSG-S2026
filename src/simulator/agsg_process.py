"""Host-side wrapper around the firmware runner process.

The firmware runs in its own interpreter (see ``agsg_shim/runner.py`` for the
protocol and the reasons). This class owns the subprocess, serialises messages
and keeps the event loop free: writes go through a queue so the websocket
handler never blocks on a slow pipe.
"""

from __future__ import annotations

import asyncio
import json
import pathlib
import sys
from typing import Callable, Optional

ROOT = pathlib.Path(__file__).resolve().parent.parent
RUNNER = ROOT / "simulator" / "agsg_shim" / "runner.py"


class FirmwareProcess:
  def __init__(
    self,
    on_message: Callable[[dict], None],
    on_exit: Optional[Callable[[int], None]] = None,
    python: Optional[str] = None,
    runner: pathlib.Path = RUNNER,
  ) -> None:
    self._on_message = on_message
    self._on_exit = on_exit
    self._python = python or sys.executable
    self._runner = runner
    self._proc: Optional[asyncio.subprocess.Process] = None
    self._queue: asyncio.Queue = asyncio.Queue()
    self._tasks = []
    self.running = False
    self.ready = False
    self.exit_note: Optional[str] = None

  async def start(self) -> None:
    self._proc = await asyncio.create_subprocess_exec(
      self._python,
      "-u",
      str(self._runner),
      stdin=asyncio.subprocess.PIPE,
      stdout=asyncio.subprocess.PIPE,
      stderr=asyncio.subprocess.PIPE,
    )
    self.running = True
    self._tasks = [
      asyncio.create_task(self._read_stdout()),
      asyncio.create_task(self._read_stderr()),
      asyncio.create_task(self._write_loop()),
    ]

  def send(self, payload: dict) -> None:
    if not self.running:
      return
    self._queue.put_nowait(payload)

  async def stop(self) -> None:
    self.running = False
    proc = self._proc
    if proc is None:
      return
    try:
      if proc.stdin and not proc.stdin.is_closing():
        proc.stdin.write(b'{"t":"stop"}\n')
        await proc.stdin.drain()
        proc.stdin.close()
    except (BrokenPipeError, ConnectionResetError, RuntimeError):
      pass
    try:
      await asyncio.wait_for(proc.wait(), timeout=1.0)
    except asyncio.TimeoutError:
      proc.kill()
      await proc.wait()
    for task in self._tasks:
      task.cancel()
    self._tasks = []
    self._proc = None
    self.ready = False

  # ------------------------------------------------------------------ internals

  async def _write_loop(self) -> None:
    proc = self._proc
    if proc is None or proc.stdin is None:
      return
    while self.running:
      payload = await self._queue.get()
      try:
        proc.stdin.write((json.dumps(payload, separators=(",", ":")) + "\n").encode())
        await proc.stdin.drain()
      except (BrokenPipeError, ConnectionResetError):
        break

  async def _read_stdout(self) -> None:
    proc = self._proc
    if proc is None or proc.stdout is None:
      return
    while True:
      line = await proc.stdout.readline()
      if not line:
        break
      text = line.decode("utf-8", "replace").strip()
      if not text:
        continue
      try:
        payload = json.loads(text)
      except json.JSONDecodeError:
        continue
      if payload.get("t") == "ready":
        self.ready = True
      self._on_message(payload)
    code = proc.returncode if proc.returncode is not None else -1
    if self.running:
      self.exit_note = "firmware process exited (code {})".format(code)
      self.running = False
      if self._on_exit:
        self._on_exit(code)

  async def _read_stderr(self) -> None:
    proc = self._proc
    if proc is None or proc.stderr is None:
      return
    tail = []
    while True:
      line = await proc.stderr.readline()
      if not line:
        break
      text = line.decode("utf-8", "replace").rstrip()
      tail.append(text)
      tail = tail[-20:]
      # Surface firmware tracebacks immediately: a dead background thread shows
      # up as a frozen pose, not as a crash.
      self._on_message({"t": "stderr", "m": text})
    if tail and self.running:
      self.exit_note = "firmware stderr: " + " | ".join(tail[-4:])
