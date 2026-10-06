import uasyncio as asyncio
from machine import Pin, UART, freq
import time

freq(int(2.5 * 100_000_000))

# Debug option: Set to True to enable debug output
DEBUG = False

# Output pacing (~50 Hz, matching the receiver on the main Pico)
SEND_INTERVAL_MS = 20
# Maximum echo wait (~510 cm round-trip; the distance cap is 250 cm)
ECHO_TIMEOUT_US = 30000

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


def fire_triggers():
    """Send a 10 us trigger pulse to all four sonars at once.

    Note: firing all sonars simultaneously maximises throughput, but if you
    see cross-talk (echoes leaking between sensors), fire them with a small
    stagger instead (a few ms between each trigger).
    """
    for t in triggers:
        t.value(0)
    time.sleep_us(2)
    for t in triggers:
        t.value(1)
    time.sleep_us(10)
    for t in triggers:
        t.value(0)


def measure_all():
    """Trigger all sonars and measure all four echoes concurrently.

    Returns [front, rear, left, right], each scaled by 10 (-1 on timeout).
    """
    fire_triggers()

    start_us = [None, None, None, None]
    end_us = [None, None, None, None]
    t0 = time.ticks_us()

    # Single tight loop polling the four echo pins at once (no yield, to keep
    # microsecond timing on the pulse widths).
    while True:
        now = time.ticks_us()
        done = True
        for i in range(4):
            v = echos[i].value()
            if v and start_us[i] is None:
                start_us[i] = now
            elif not v and start_us[i] is not None and end_us[i] is None:
                end_us[i] = now
            if end_us[i] is None:
                done = False
        if done or time.ticks_diff(now, t0) > ECHO_TIMEOUT_US:
            break

    distances = []
    for i in range(4):
        if start_us[i] is not None and end_us[i] is not None:
            pulse_us = time.ticks_diff(end_us[i], start_us[i])
            cm = pulse_us / 58.0
            scaled = int(round(cm * 10))
            distances.append(2500 if scaled > 2500 else scaled)
        else:
            distances.append(-1)

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

        # Pace the loop to a steady output rate.
        elapsed = time.ticks_diff(time.ticks_ms(), t0)
        wait = SEND_INTERVAL_MS - elapsed
        if wait > 0:
            await asyncio.sleep_ms(wait)


asyncio.run(main())
