# SonarSlave debug tool.
#
# Run this instead of main.py to verify the four sonars and the UART framing.
# It measures each sonar individually (sequential, no cross-talk), then all
# four at once (concurrent, like main.py), and prints the exact bytes that
# would be sent over UART.
#
# How to run: temporarily rename this file to main.py, or in the REPL run:
#     import debug

from machine import Pin, UART, freq, time_pulse_us
import time

freq(int(2.5 * 100_000_000))

# Send the packet over UART too (True), or just print it (False).
SEND_OVER_UART = False

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
labels = ["front", "rear", "left", "right"]

uart = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(17))


def measure_one(trigger, echo):
    """Measure a single sonar sequentially (accurate, no cross-talk).

    Returns the distance scaled by 10, or -1 on timeout/no echo.
    """
    trigger.value(0)
    time.sleep_us(2)
    trigger.value(1)
    time.sleep_us(10)
    trigger.value(0)

    # time_pulse_us returns the high-pulse width in us, or a negative value
    # on timeout (-2: no pulse started, -1: pulse never ended).
    pulse_us = time_pulse_us(echo, 1, 30000)
    if pulse_us < 0:
        return -1

    cm = pulse_us / 58.0
    scaled = int(round(cm * 10))
    return 2500 if scaled > 2500 else scaled


def measure_all():
    """Fire all four sonars at once and measure them concurrently (like main.py)."""
    for t in triggers:
        t.value(0)
    time.sleep_us(2)
    for t in triggers:
        t.value(1)
    time.sleep_us(10)
    for t in triggers:
        t.value(0)

    start_us = [None, None, None, None]
    end_us = [None, None, None, None]
    t0 = time.ticks_us()

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
        if done or time.ticks_diff(now, t0) > 30000:
            break

    out = []
    for i in range(4):
        if start_us[i] is not None and end_us[i] is not None:
            cm = time.ticks_diff(end_us[i], start_us[i]) / 58.0
            s = int(round(cm * 10))
            out.append(2500 if s > 2500 else s)
        else:
            out.append(-1)
    return out


def pack_packet(distances):
    """Encode: header 0xA5 + 4 x uint16 LE + XOR checksum."""
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


def fmt(d):
    return "-----" if d < 0 else "{:5.1f} cm".format(d / 10.0)


def hexdump(pkt):
    return " ".join("{:02X}".format(b) for b in pkt)


print("=== SonarSlave debug ===")

while True:
    seq = [measure_one(triggers[i], echos[i]) for i in range(4)]
    conc = measure_all()

    print("sequential | concurrent")
    print("-----------|-----------")
    for i in range(4):
        print("{:5s} {:>8s} | {:>8s}".format(labels[i], fmt(seq[i]), fmt(conc[i])))

    pkt = pack_packet(conc)
    print("UART ({} bytes): {}".format(len(pkt), hexdump(pkt)))
    print("header=0x{:02X}  checksum=0x{:02X}".format(pkt[0], pkt[-1]))

    # Flag readings where concurrent differs a lot from sequential (cross-talk).
    for i in range(4):
        if seq[i] > 0 and conc[i] > 0 and abs(seq[i] - conc[i]) > 100:  # > 10 cm
            print("WARN: {} differs >10 cm (seq {} vs conc {}) -> possible cross-talk".format(
                labels[i], fmt(seq[i]), fmt(conc[i])))

    if SEND_OVER_UART:
        uart.write(pkt)

    print("---")
    time.sleep_ms(500)
