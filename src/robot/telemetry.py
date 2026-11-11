from uart import Uart
from state import state
from distance import Distance

try:
    import os as _os
except ImportError:
    _os = None

try:
    import sys as _sys
except ImportError:
    _sys = None


def _is_micropython():
    """True on the car, False in the simulator.

    `sys.implementation.name` is "micropython" on every MicroPython port and
    "cpython" here, and it is documented on the board, so it is the cheapest
    honest way to tell the two apart. getattr everywhere: the attribute is not
    guaranteed on every build.
    """
    implementation = getattr(_sys, "implementation", None)
    return getattr(implementation, "name", "") == "micropython"


def _environment_value():
    """WRO_TELEMETRY as a string, or None when there is no such thing.

    The board has no process environment (MicroPython's os has no environ), so
    there this is always None and the class attribute is the switch.
    """
    if _os is None:
        return None
    try:
        return _os.environ.get("WRO_TELEMETRY")  # mpy-ok
    except AttributeError:
        return None


def _initial_enabled():
    """Telemetry starts ON in the simulator and OFF on the car.

    On the car the UART is shared with the sonar frame at 115200 baud (~11.5
    kB/s) and every write blocks once the ring is full: update_odom() alone logs
    two long lines 40 times a second, so the control loop ends up waiting on the
    link. The simulator has no such limit, and those lines are what the UI log
    and the exported round show, so there it stays on.

    WRO_TELEMETRY overrides both directions: 0/off/false/no silences it, any
    other value turns it back on (that is also how to get the log back on the
    bench without editing this file, on a port that does have an environment).
    """
    value = _environment_value()
    if value is not None:
        return str(value).strip().lower() not in ("0", "false", "off", "no")
    return not _is_micropython()


class Telemetry:
    """
    Static telemetry helper.

    Formats and transmits pose and distance data over the shared UART
    interface. The web UI expects CSV lines in the form:

        x,y,headingDeg,front,rear,left,right*CS

    where all distances are in metres, heading is degrees CCW from +X,
    and CS is an XOR checksum (two hex chars) over the ASCII payload
    before the '*'.

    ENABLED is the master switch for everything this class writes, and it starts
    OFF on the car and ON in the simulator (see _initial_enabled). To get the
    lines back on the board while debugging, set ENABLED = True here, or start
    the simulator with WRO_TELEMETRY=1 to be explicit. Silencing it costs the
    firmware's own lines in the UI log and in the exported round; the sensor
    stream to the simulator is not affected.
    """

    ENABLED = _initial_enabled()

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
        if not Telemetry.ENABLED:
            return
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
            360 - ((state.compass_angle + (180 if state.clockwise == 1 else 0)) % 360),
            front, rear, left, right,
        )
        checksum = Telemetry._xor_checksum(payload)
        uart.write("{}*{:02X}\n".format(payload, checksum))

    @staticmethod
    def log(message) -> None:
        """Send a plain-text log line over UART (shown in the UI log panel)."""
        if not Telemetry.ENABLED:
            return
        uart = getattr(Uart, "_uart", None)
        if uart is None:
            return
        uart.write("{}\n".format(str(message)))

    @staticmethod
    def log_ekf(trace, innov_x, innov_y) -> None:
        """Plain-text pose-filter status. Not part of the CSV pose frame."""
        def _fmt(value):
            if value is None:
                return "none"
            return "%.2f" % value

        Telemetry.log(
            "ekf trace=%.2f innov_x=%s innov_y=%s"
            % (trace, _fmt(innov_x), _fmt(innov_y))
        )
