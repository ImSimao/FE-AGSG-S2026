from uart import Uart
from state import state
from distance import Distance


class Telemetry:
    """
    Static telemetry helper.

    Formats and transmits pose and distance data over the shared UART
    interface. The web UI expects CSV lines in the form:

        x,y,headingDeg,front,rear,left,right*CS

    where all distances are in metres, heading is degrees CCW from +X,
    and CS is an XOR checksum (two hex chars) over the ASCII payload
    before the '*'.
    """

    @staticmethod
    def _xor_checksum(payload: str) -> int:
        """Return XOR of all ASCII bytes in payload."""
        checksum = 0
        for b in payload.encode("ascii"):
            checksum ^= b
        return checksum & 0xFF

    @staticmethod
    def send() -> None:
        """Format and push one telemetry frame over UART using current state."""
        uart = getattr(Uart, "_uart", None)
        if uart is None:
            return

        d = Distance._last_sensor_data
        front = max(d.get("front", 0.0), 0.0) / 100.0
        rear  = max(d.get("rear",  0.0), 0.0) / 100.0
        left  = max(d.get("left",  0.0), 0.0) / 100.0
        right = max(d.get("right", 0.0), 0.0) / 100.0

        payload = "{:.3f},{:.3f},{:.1f},{:.2f},{:.2f},{:.2f},{:.2f}".format(
            state.odom_x / 100.0,
            state.odom_y / 100.0,
            state.compass_angle,
            front, rear, left, right,
        )
        checksum = Telemetry._xor_checksum(payload)
        uart.write("{}*{:02X}\n".format(payload, checksum))

    @staticmethod
    def log(message) -> None:
        """Send a plain-text log line over UART (shown in the UI log panel)."""
        uart = getattr(Uart, "_uart", None)
        if uart is None:
            return
        uart.write("{}\n".format(str(message)))
