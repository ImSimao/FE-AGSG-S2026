from machine import Pin
import math


# ===== CONFIGURATION =====
PPR = 200               # encoder pulses per revolution
RATIO = 3 / 5           # gear ratio
WHEEL_DIAMETER = 3.2    # centimetres

# Pins (static)
_encoder_a = Pin(7, Pin.IN, Pin.PULL_UP)
_encoder_b = Pin(6, Pin.IN, Pin.PULL_UP)

# State — module global for fast access inside the IRQ
_count = 0


def _irq(pin):
    global _count
    # On the rising edge of channel A, the level of B indicates the direction.
    if _encoder_b.value():
        _count += 1   # forward
    else:
        _count -= 1   # backward


# Interrupt on the rising edge of channel A
_encoder_a.irq(trigger=Pin.IRQ_RISING, handler=_irq)


class Encoder:
    """Wrapper that keeps the old API (Encoder.distance_cm())."""

    @staticmethod
    def distance_cm():
        return _count / (PPR / RATIO) * (WHEEL_DIAMETER * math.pi)
