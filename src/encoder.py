from machine import Pin
import time
import math


# ===== CONFIGURATION =====
PPR = 200               # encoder pulses per revolution
RATIO = 3 / 5           # gear ratio
WHEEL_DIAMETER = 3.2    # centimetres

# Dynamic debounce (adjusted by speed):
MIN_DEBOUNCE_US = 20    # high speed (closely-spaced pulses)
MAX_DEBOUNCE_US = 1500  # low speed (widely-spaced pulses)

# Pins (static)
_encoder_a = Pin(7, Pin.IN, Pin.PULL_UP)
_encoder_b = Pin(6, Pin.IN, Pin.PULL_UP)

# State — module globals for fast access inside the IRQ
_count = 0
_last_edge_us = 0
_debounce_us = 200  # initial value


def update_debounce_from_speed(speed_cm_s):
    """Adjusts the debounce to the expected encoder period (1/4 of the period)."""
    global _debounce_us
    if speed_cm_s <= 0.5:
        _debounce_us = MAX_DEBOUNCE_US
        return
    # expected period (us) = 1e6 / (counts/s)
    # counts/s = speed * (PPR/RATIO) / circumference
    # => period ~= 30158 / speed (cm/s)
    period_us = 30158.0 / speed_cm_s
    debounce = int(period_us / 4)
    if debounce < MIN_DEBOUNCE_US:
        debounce = MIN_DEBOUNCE_US
    elif debounce > MAX_DEBOUNCE_US:
        debounce = MAX_DEBOUNCE_US
    _debounce_us = debounce


def _irq(pin):
    global _count, _last_edge_us
    now = time.ticks_us()
    if time.ticks_diff(now, _last_edge_us) < _debounce_us:
        return  # bounce/noise, ignore
    _last_edge_us = now
    # On the rising edge of channel A, the level of B indicates the direction.
    if _encoder_b.value():
        _count -= 1   # Backward
    else:
        _count += 1   # Forward


# Interrupt on the rising edge of channel A
_encoder_a.irq(trigger=Pin.IRQ_RISING, handler=_irq)


class Encoder:
    """Wrapper that keeps the old API (Encoder.distance_cm())."""

    @staticmethod
    def distance_cm():
        return _count / (PPR / RATIO) * (WHEEL_DIAMETER * math.pi)
