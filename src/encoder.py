from machine import Pin
import time
import math


# ===== CONFIGURATION =====
PPR = 200               # encoder pulses per revolution
RATIO = 3 / 5           # gear ratio
WHEEL_DIAMETER = 3.2    # centimetres

# Fixed debounce: filters the 20 kHz PWM noise (~50 us) while keeping the real
# encoder edges (period >= ~300 us at maximum motor speed). It must NEVER be set
# large enough to reject real edges, or the encoder stops counting entirely.
DEBOUNCE_US = 200

# Pins (static)
_encoder_a = Pin(7, Pin.IN, Pin.PULL_UP)
_encoder_b = Pin(6, Pin.IN, Pin.PULL_UP)

# State — module globals for fast access inside the IRQ
_count = 0
_last_edge_us = 0


def _irq(pin):
    global _count, _last_edge_us
    now = time.ticks_us()
    if time.ticks_diff(now, _last_edge_us) < DEBOUNCE_US:
        return  # bounce/noise, ignore
    _last_edge_us = now
    # On the rising edge of channel A, the level of B indicates the direction.
    if _encoder_b.value():
        _count -= 1   # backward
    else:
        _count += 1   # forward


# Interrupt on the rising edge of channel A
_encoder_a.irq(trigger=Pin.IRQ_RISING, handler=_irq)


class Encoder:
    """Wrapper that keeps the old API (Encoder.distance_cm())."""

    @staticmethod
    def distance_cm():
        return _count / (PPR / RATIO) * (WHEEL_DIAMETER * math.pi)
