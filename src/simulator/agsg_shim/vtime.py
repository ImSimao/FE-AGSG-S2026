"""A MicroPython ``time`` module whose clock runs at the simulator's speed.

The firmware runs its control loop on real time: ``time.sleep_ms(20)`` is 20 ms
of its own clock, and its PID integrates against ``ticks_diff`` of that clock.
When the world runs at 2x, the car covers twice the ground per firmware tick, so
the same PID sees a vehicle moving twice as fast: it lags and the car zigzags.
That is not a tuning problem, it is a different vehicle.

So the speed multiplier has to live on both sides. Everything the firmware calls
on ``time`` is expressed in *virtual* microseconds that advance ``scale`` times
faster than the wall clock, and sleeps are shortened by the same factor. A 2x run
is then a time lapse of the same race, not a faster car.
"""

import os as _os
import sys as _sys
import time as _real_time

_TICKS_PERIOD = 1 << 30

# Debug aid: WRO_SLEEP_LOG=1 prints every firmware sleep long enough to matter,
# with the wall time it actually took. A sleep whose wall duration does not
# shrink with the multiplier is a bug in the time-lapse, not in the firmware.
_LOG_SLEEPS = _os.environ.get("WRO_SLEEP_LOG") == "1"


class VirtualClock:
  def __init__(self, scale: float = 1.0) -> None:
    self._scale = max(0.05, float(scale))
    self._t0 = _real_time.monotonic()
    self._base = 0.0
    # When the controller feeds the simulated clock, the firmware's time *is*
    # the world's time. Deriving it from wall time (below) means every millisecond
    # the browser, the websocket or the operating system loses shows up as the
    # car having moved a different distance per control step - and at 4x a 20 ms
    # sleep becomes a 5 ms sleep, which the OS rounds up, so the control period
    # stretched in simulated time and the same seed drove differently.
    self._sim_ms = None
    self._stalls = 0
    # Called with True when the firmware blocks in a sleep and False when it
    # wakes. The controller uses it to step the world: one physics step per
    # sleep, so the firmware samples the same instants in the same order in every
    # run instead of whenever the browser happened to tick.
    self.on_wait = None

  @property
  def scale(self) -> float:
    return self._scale

  def sync_to_sim(self, sim_ms: float) -> None:
    """Take the time from the simulator instead of from the wall clock."""
    self._sim_ms = float(sim_ms)

  def now(self) -> float:
    """Seconds on the firmware's clock: the world's clock when it is available."""
    if self._sim_ms is not None:
      return self._sim_ms / 1000.0
    return self._base + (_real_time.monotonic() - self._t0) * self._scale

  def set_scale(self, scale: float) -> None:
    scale = max(0.05, float(scale))
    if _LOG_SLEEPS:
      _sys.stderr.write("[scale] %.1f -> %.1f (pid %d)\n" % (self._scale, scale, _os.getpid()))
      _sys.stderr.flush()
    if abs(scale - self._scale) < 1e-6:
      return
    self._base = self.now()  # freeze virtual time at the switch
    self._t0 = _real_time.monotonic()
    self._scale = scale

  # -- helpers -------------------------------------------------------------
  def sleep(self, seconds: float) -> None:
    seconds = max(0.0, float(seconds))
    if self._sim_ms is not None:
      self._sleep_on_sim(seconds)
      return
    if not (_LOG_SLEEPS and seconds >= 0.005):
      _real_time.sleep(seconds / self._scale)
      return
    t0 = _real_time.monotonic()
    _real_time.sleep(seconds / self._scale)
    took = (_real_time.monotonic() - t0) * 1000.0
    _sys.stderr.write("[sleep] asked %.0f ms at %.1fx -> %.0f ms wall\n" % (seconds * 1000.0, self._scale, took))
    _sys.stderr.flush()

  def _sleep_on_sim(self, seconds: float) -> None:
    """Block until the simulated clock has advanced by `seconds`.

    Polling, but on the simulated clock: the firmware wakes on the same tick of
    the world whatever the multiplier is, so a run is a function of the world it
    saw and not of how fast the machine happened to draw frames.
    """
    target = self._sim_ms + seconds * 1000.0
    if self.on_wait is not None:
      self.on_wait(True)
    # Wall-clock bail-out only for a stalled world (browser paused, round over):
    # it must never fire in normal running, and it says so when it does.
    deadline = _real_time.monotonic() + max(2.0, (seconds / max(self._scale, 0.05)) * 4.0 + 2.0)
    while self._sim_ms is not None and self._sim_ms < target:
      if _real_time.monotonic() > deadline:
        self._stalls += 1
        if self._stalls <= 3:
          _sys.stderr.write(
            "[vtime] simulated clock paused (requested %.0f ms, sim %.0f ms); sleeping in real time\n"
            % (seconds * 1000.0, self._sim_ms))
          _sys.stderr.flush()
        _real_time.sleep(seconds / self._scale)
        return
      _real_time.sleep(0.0005)
    if self.on_wait is not None:
      self.on_wait(False)


clock = VirtualClock()


def set_scale(scale: float) -> None:
  clock.set_scale(scale)


# MicroPython time API, in virtual time ------------------------------------

def sleep(seconds: float) -> None:
  clock.sleep(seconds)


def sleep_ms(ms: int) -> None:
  clock.sleep(float(ms) / 1000.0)


def sleep_us(us: int) -> None:
  clock.sleep(float(us) / 1000000.0)


def ticks_ms() -> int:
  return int(clock.now() * 1000.0) & (_TICKS_PERIOD - 1)


def ticks_us() -> int:
  return int(clock.now() * 1000000.0) & (_TICKS_PERIOD - 1)


def ticks_add(ticks: int, delta: int) -> int:
  return (int(ticks) + int(delta)) & (_TICKS_PERIOD - 1)


def ticks_diff(ticks1: int, ticks2: int) -> int:
  diff = (int(ticks1) - int(ticks2)) & (_TICKS_PERIOD - 1)
  if diff >= (_TICKS_PERIOD >> 1):
    diff -= _TICKS_PERIOD
  return diff


def time() -> float:
  return _real_time.time() + clock.now()


def time_ns() -> int:
  return int(clock.now() * 1e9)


def as_module():
  """A drop-in ``time`` module for the firmware: virtual clock, real fallbacks."""
  import types

  mod = types.ModuleType("time")
  for name in (
    "sleep", "sleep_ms", "sleep_us", "ticks_ms", "ticks_us", "ticks_diff",
    "ticks_add", "time", "time_ns", "clock",
  ):
    setattr(mod, name, globals()[name])
  mod.ticks_cpu = ticks_us

  def _fallback(name):
    return getattr(_real_time, name)

  mod.__getattr__ = _fallback
  return mod


class _ClockAlias:
  def __call__(self) -> float:
    return clock.now()


clock_now = _ClockAlias()
