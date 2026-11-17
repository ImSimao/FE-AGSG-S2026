import uasyncio as asyncio
from machine import Pin, UART, freq
import time

freq(int(2.5 * 100_000_000))

# Debug option: Set to True to enable debug output
DEBUG = False

# The four sonars are measured one at a time (sequential sweep). Firing them
# together risks one sensor picking up another's echo (cross-talk), and a
# cross-talk spike reads as a wall a few centimetres away. Measuring one at a
# time costs time, because a sweep is the SUM of the four echoes and not the
# longest one:
#
#   corridor distances (1-2 m each, 6-12 ms):  4 x ~10 ms + 3 x 3 ms -> ~50 ms (~20 Hz)
#   worst case (every echo times out):         4 x 30 ms + 3 x 3 ms  -> ~129 ms (~8 Hz)
#
# The main Pico polls the frame at 50 Hz and keeps the last values, so a sweep
# slower than 20 ms simply means the frame arrives as soon as it is ready.

# Inactivity floor between frames (the receiver polls at 50 Hz).
SEND_INTERVAL_MS = 20
# Echo wait per sensor (~510 cm round-trip; the distance cap is 250 cm).
ECHO_TIMEOUT_US = 30000
# Quiet gap after each echo, so the previous burst has died away before the
# next sensor fires. Raise it if debug.py still flags cross-talk.
SENSOR_GAP_MS = 3

# SRF04 GPIO pins in order: front, rear, left, right
echos = [
    Pin(8, Pin.IN, pull=Pin.PULL_DOWN),   # front
    Pin(12, Pin.IN, pull=Pin.PULL_DOWN),  # rear
    Pin(15, Pin.IN, pull=Pin.PULL_DOWN),  # left
    Pin(10, Pin.IN, pull=Pin.PULL_DOWN),  # right
]
triggers = [
    Pin(9, Pin.OUT),   # front
    Pin(7, Pin.OUT),   # rear
    Pin(14, Pin.OUT),  # left
    Pin(11, Pin.OUT),  # right
]
sensor_labels = ["front", "rear", "left", "right"]

uart = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(17))


def wait_quiet(echo, timeout_us=5000):
    """Wait for the echo pin to go low before firing that sonar.

    A pulse still high when we start would be counted as our own echo, which
    reads as a wall a few centimetres away. Bounded so a stuck pin cannot hang
    the sweep.
    """
    t0 = time.ticks_us()
    while echo.value():
        if time.ticks_diff(time.ticks_us(), t0) > timeout_us:
            return False
    return True


def fire(i):
    """Send one 10 us trigger pulse to sonar i."""
    trigger = triggers[i]
    trigger.value(0)
    time.sleep_us(2)
    trigger.value(1)
    time.sleep_us(10)
    trigger.value(0)


def measure_one(i):
    """Fire sonar i on its own and time its echo.

    Returns the distance scaled by 10 (cm), or -1 when no echo came back.
    """
    echo = echos[i]
    wait_quiet(echo)
    fire(i)

    start_us = None
    end_us = None
    t0 = time.ticks_us()

    # Tight poll on this one echo pin, no yield: a 1 cm change is 58 us of
    # pulse width, so the loop must stay in the microsecond range.
    while True:
        now = time.ticks_us()
        if echo.value():
            if start_us is None:
                start_us = now
        elif start_us is not None:
            end_us = now
            break
        if time.ticks_diff(now, t0) > ECHO_TIMEOUT_US:
            break

    if start_us is None or end_us is None:
        return -1

    pulse_us = time.ticks_diff(end_us, start_us)
    cm = pulse_us / 58.0
    scaled = int(round(cm * 10))
    return 2500 if scaled > 2500 else scaled


def measure_all():
    """Sweep the four sonars one at a time: [front, rear, left, right]."""
    distances = []
    for i in range(4):
        distances.append(measure_one(i))
        if i < 3:
            time.sleep_ms(SENSOR_GAP_MS)
    return distances


def pack_packet(distances):
    """Encode distances as: header 0xA5 + 4 x uint16 LE + XOR checksum."""
    data = bytearray()
    for d in distances:
        if d < 0:
            data += b"\xFF\xFF"
        else:
            data += bytes([d & 0xFF, (d >> 8) & 0xFF])

    checksum = 0
    for b in data:
        checksum ^= b

    return b"\xA5" + data + bytes([checksum])


async def main():
    while True:
        t0 = time.ticks_ms()

        distances = measure_all()

        if DEBUG:
            cm = [d / 10.0 if d != -1 else -1 for d in distances]
            print("Distances (cm):", cm)

        uart.write(pack_packet(distances))

        # Pace the loop to a steady output rate. A sequential sweep usually
        # takes longer than SEND_INTERVAL_MS, in which case there is no wait.
        elapsed = time.ticks_diff(time.ticks_ms(), t0)
        wait = SEND_INTERVAL_MS - elapsed
        if wait > 0:
            await asyncio.sleep_ms(wait)


asyncio.run(main())
