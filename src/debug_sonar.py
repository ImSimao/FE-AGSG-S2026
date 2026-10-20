# debug_sonar.py — measure the sonar protocol frame-loss rate.
#
# Reads the distance UART and counts valid frames (header 0xA5 + correct XOR
# checksum) versus invalid frames, so the frame-loss rate can be published in
# the journal §4.9.
#
# How to run: rename this file to main.py, or in the REPL: import debug_sonar

import time
from machine import freq
from uart import Uart

freq(int(3 * 100_000_000))
Uart.initialize()
uart = Uart._uart

HEADER = 0xA5
PACKET_LEN = 10
DURATION_MS = 10000  # sample window (10 s)

buf = b""
valid = 0
invalid = 0
start = time.ticks_ms()

print("Measuring sonar frame loss for %.0f s..." % (DURATION_MS / 1000))

while time.ticks_diff(time.ticks_ms(), start) < DURATION_MS:
    if uart.any():
        data = uart.read(uart.any())
        if data:
            buf += data
            buf = buf[-64:]

    while True:
        idx = buf.find(bytes([HEADER]))
        if idx < 0:
            buf = buf[-(PACKET_LEN - 1):]
            break
        if idx > 0:
            buf = buf[idx:]
        if len(buf) < PACKET_LEN:
            break
        body = buf[1:9]
        checksum = buf[9]
        calc = 0
        for b in body:
            calc ^= b
        if calc == checksum:
            valid += 1
        else:
            invalid += 1
        buf = buf[PACKET_LEN:]

    time.sleep_ms(1)

total = valid + invalid
loss = (invalid / total * 100.0) if total else 0.0
print("=== Sonar protocol result ===")
print("valid frames:  %d" % valid)
print("invalid frames:%d" % invalid)
print("total frames:  %d" % total)
print("frame loss:    %.1f%%" % loss)
