"""Minimal MicroPython ``machine`` module for CPython.

Only what the AGSG firmware touches is implemented: ``Pin`` (IN/OUT/pull/irq),
``PWM`` (freq/duty_u16), ``UART`` (any/read/write), ``I2C`` (scan only) and
``freq``. Everything else raises AttributeError, which is the point: if the
firmware starts using a peripheral nobody has shimmed, the run fails loudly
instead of silently inventing behaviour.
"""

from __future__ import annotations

from hardware import hal


class Pin:
  IN = 0
  OUT = 1
  OPEN_DRAIN = 2
  PULL_UP = 1
  PULL_DOWN = 2
  IRQ_RISING = 1
  IRQ_FALLING = 2

  def __init__(self, id, mode=-1, pull=None, value=None):
    self.id = id
    self.mode = mode
    self.pull = pull
    if value is not None:
      self._level = 1 if value else 0
    else:
      self._level = 0
    hal.levels.setdefault(id, 0)

  # the firmware drives outputs through on/off, value(), or the call itself
  def value(self, val=None):
    if val is None:
      return self._level
    self._level = 1 if val else 0
    hal.levels[self.id] = self._level

  def on(self):
    self.value(1)

  def off(self):
    self.value(0)

  def toggle(self):
    self.value(0 if self._level else 1)

  def __call__(self, val=None):
    return self.value(val)

  def irq(self, handler=None, trigger=None, **kwargs):
    hal.irq_handlers[self.id] = handler

  def init(self, *args, **kwargs):
    pass


class PWM:
  def __init__(self, dest, freq=None, duty_u16=None, duty_ns=None):
    self.pin = dest if isinstance(dest, Pin) else Pin(dest)
    self.freq_hz = freq or 0
    self.duty = 0.0  # 0..1, normalised
    hal.pwm[self.pin.id] = self
    if duty_u16 is not None:
      self.duty_u16(duty_u16)
    if duty_ns is not None:
      self.duty_ns(duty_ns)

  def freq(self, value=None):
    if value is None:
      return self.freq_hz
    self.freq_hz = value

  def duty_u16(self, value=None):
    if value is None:
      return int(self.duty * 65535)
    self.duty = max(0.0, min(1.0, float(value) / 65535.0))

  def duty_ns(self, value=None):
    period_ns = (1e9 / self.freq_hz) if self.freq_hz else 20_000_000.0
    if value is None:
      return int(self.duty * period_ns)
    self.duty = max(0.0, min(1.0, float(value) / period_ns))

  def deinit(self):
    pass


class UART:
  """One object per UART id; reads and writes are independent.

  On the board UART0 carries the sonar slave in on RX and the HC-06 debug link
  out on TX, so the firmware both reads and writes the same object.
  """

  def __init__(self, id, baudrate=115200, tx=None, rx=None, **kwargs):
    self.id = id
    self.baudrate = baudrate
    self.tx = tx
    self.rx = rx
    hal.rx.setdefault(id, bytearray())

  def any(self):
    return len(hal.rx.get(self.id, b""))

  def read(self, n=None):
    buf = hal.rx.get(self.id)
    if not buf:
      return None
    if n is None or n >= len(buf):
      data = bytes(buf)
      hal.rx[self.id] = bytearray()
      return data
    data = bytes(buf[:n])
    hal.rx[self.id] = bytearray(buf[n:])
    return data

  def readline(self):
    buf = hal.rx.get(self.id)
    if not buf:
      return None
    idx = buf.find(b"\n")
    if idx < 0:
      return None
    data = bytes(buf[: idx + 1])
    hal.rx[self.id] = bytearray(buf[idx + 1 :])
    return data

  def readinto(self, buf):
    data = self.read(len(buf))
    if not data:
      return None
    buf[: len(data)] = data
    return len(data)

  def write(self, data):
    if isinstance(data, str):
      data = data.encode("utf-8")
    for line in bytes(data).split(b"\n"):
      piece = line.decode("ascii", "ignore").strip()
      if piece:
        hal.tx.append(piece)
    return len(data)

  def init(self, *args, **kwargs):
    pass

  def deinit(self):
    pass


class I2C:
  def __init__(self, id, scl=None, sda=None, freq=400_000, timeout=None, **kwargs):
    self.id = id
    self.freq_hz = freq

  def scan(self):
    return [0x4A]  # BNO085

  def readfrom(self, addr, nbytes, stop=True):
    return bytes(nbytes)

  def writeto(self, addr, buf, stop=True):
    return len(buf)


class ADC:
  def __init__(self, pin, **kwargs):
    self.pin = pin

  def read_u16(self):
    return 0


def freq(hz=None):
  if hz is None:
    return hal.cpu_hz
  hal.cpu_hz = hz
