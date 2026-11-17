# SonarSlave debug tool.
#
# Run this instead of main.py to verify the four sonars and the UART framing.
# It measures the four sonars one at a time (sequential, the mode main.py
# uses), then all four at once (concurrent, the mode we left), and prints the
# exact bytes that would be sent over UART.
#
# Why both: concurrent firing is faster but let one sensor hear another's
# echo. If the two columns agree on the bench, the cross-talk is not present
# in this environment; where they disagree, it is, and sequential is earning
# its cost. The sequential column includes the same guard gap as main.py, so
# the comparison matches what the car actually sends.
#
# How to run: temporarily rename this file to main.py, or in the REPL run:
#     import debug

from machine import Pin, UART, freq, time_pulse_us
import time

freq(int(2.5 * 100_000_000))

# Send the packet over UART too (True), or just print it (False).
SEND_OVER_UART = False

# Keep in sync with main.py.
ECHO_TIMEOUT_US = 30000
SENSOR_GAP_MS = 3

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
    """Measure a single sonar on its own (sequential, like main.py).

    Returns the distance scaled by 10, or -1 on timeout/no echo.
    """
    # A pulse still high here is the tail of the previous burst, not our echo.
    t0 = time.ticks_us()
    while echo.value():
        if time.ticks_diff(time.ticks_us(), t0) > 5000:
            break

    trigger.value(0)
    time.sleep_us(2)
    trigger.value(1)
    time.sleep_us(10)
    trigger.value(0)

    # time_pulse_us returns the high-pulse width in us, or a negative value
    # on timeout (-2: no pulse started, -1: pulse never ended).
    pulse_us = time_pulse_us(echo, 1, ECHO_TIMEOUT_US)
    if pulse_us < 0:
        return -1

    cm = pulse_us / 58.0
    scaled = int(round(cm * 10))
    return 2500 if scaled > 2500 else scaled


def measure_seq_all():
    """Sweep the four sonars one at a time, with the guard gap main.py uses."""
    out = []
    for i in range(4):
        out.append(measure_one(triggers[i], echos[i]))
        if i < 3:
            time.sleep_ms(SENSOR_GAP_MS)
    return out


def measure_all():
    """Fire all four sonars at once and measure them concurrently (the old mode)."""
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
        if done or time.ticks_diff(now, t0) > ECHO_TIMEOUT_US:
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
    seq = measure_seq_all()
    conc = measure_all()

    print("sequential | concurrent")
    print("-----------|-----------")
    for i in range(4):
        print("{:5s} {:>8s} | {:>8s}".format(labels[i], fmt(seq[i]), fmt(conc[i])))

    pkt = pack_packet(seq)
    print("UART ({} bytes): {}".format(len(pkt), hexdump(pkt)))
    print("header=0x{:02X}  checksum=0x{:02X}".format(pkt[0], pkt[-1]))

    # Flag readings where the concurrent pass differs a lot (cross-talk). This
    # is the measurement that justified going sequential.
    for i in range(4):
        if seq[i] > 0 and conc[i] > 0 and abs(seq[i] - conc[i]) > 100:  # > 10 cm
            print("WARN: {} differs >10 cm (seq {} vs conc {}) -> cross-talk confirmed".format(
                labels[i], fmt(seq[i]), fmt(conc[i])))

    if SEND_OVER_UART:
        uart.write(pkt)

    print("---")
    time.sleep_ms(500)
