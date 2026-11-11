import math
import time
import machine

# Web UI expects CSV lines: x,y,headingDeg[,front,rear,left,right]*CS
# All distances are meters; heading is degrees CCW from +X.
# CS is an XOR checksum (two hex chars) over the ASCII payload before "*".
uart1 = machine.UART(0, baudrate=115200, tx=machine.Pin(0), rx=machine.Pin(1))


def _xor_checksum(payload: str) -> int:
    """Return XOR of all ASCII bytes in payload."""
    checksum = 0
    for b in payload.encode("ascii"):
        checksum ^= b
    return checksum & 0xFF


def send_telemetry(x_m, y_m, heading_deg, distances):
    """Format and push one telemetry line over UART1 (with checksum)."""
    front = distances.get("Front", 0.0)
    rear = distances.get("Rear", 0.0)
    left = distances.get("Left", 0.0)
    right = distances.get("Right", 0.0)
    payload = "{:.3f},{:.3f},{:.1f},{:.2f},{:.2f},{:.2f},{:.2f}".format(
        x_m, y_m, heading_deg, front, rear, left, right
    )
    checksum = _xor_checksum(payload)
    line = "{}*{:02X}\n".format(payload, checksum)
    uart1.write(line)


def demo_telemetry_loop(period_s=(1/60)):
    """
    Emit a smooth, moving pose plus fake sonar readings.

    Replace the generated values with real sensors/odom as you integrate.
    """
    start_ms = time.ticks_ms()
    while True:
        t = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        x = 0.5 + 0.25 * math.sin(t * 0.4)
        y = 1.0 + 0.25 * math.cos(t * 0.4)
        heading = (t * 30.0) % 360.0
        distances = {
            "Front": 0.30 + 0.05 * math.sin(t * 0.6),
            "Rear": 0.35 + 0.04 * math.cos(t * 0.5),
            "Left": 0.25 + 0.03 * math.sin(t * 0.7),
            "Right": 0.28 + 0.03 * math.cos(t * 0.8),
        }
        send_telemetry(x, y, heading, distances)
        time.sleep(period_s)


demo_telemetry_loop()