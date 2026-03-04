
from uart import Uart


class Distance:
    """
    Static helper for reading distance sensors over UART.

    Expects a single 8‑byte packet formatted as 4 little‑endian 16‑bit integers
    (front, rear, left, right), each value scaled by 10 (1 decimal place).
    """

    # Cached last known distances (in cm)
    _last_sensor_data = {
        "front": -1.0,
        "rear": -1.0,
        "left": -1.0,
        "right": -1.0,
    }

    @staticmethod
    def _unpack_distance(byte_data: bytes, offset: int) -> float:
        """Unpack a 16‑bit little‑endian integer and convert to cm."""
        low_byte = byte_data[offset]
        high_byte = byte_data[offset + 1]
        scaled_value = low_byte | (high_byte << 8)

        # Error value from sensor
        if scaled_value == 0xFFFF:
            return -1.0

        return scaled_value / 10.0

    @staticmethod
    def _read_packet() -> bytes | None:
        """
        Read a single 8‑byte packet from UART, if available.

        Uses the shared UART instance from `Uart`. If UART is not
        initialized or there is not enough data, returns None.
        """
        uart = getattr(Uart, "_uart", None)
        if uart is None:
            return None

        try:
            # Only try to read if there is data waiting
            available = uart.any()
            if not available or available < 8:
                return None

            data = uart.read(available)
            if not data or len(data) < 8:
                return None

            packet = data[-8:]

            # Uncomment for debugging raw UART data
            # print("UART raw:", data, "packet:", packet)

            return packet
        except Exception as e:
            print(f"UART read error (distance): {e}")
            return None

    @staticmethod
    def get_sensor_data() -> dict:
        """
        Get the latest distance values.

        Tries to read one fresh packet from UART. If successful,
        updates the cached values. Always returns the last known
        values (in cm).
        """
        try:
            packet = Distance._read_packet()

            if packet is not None:
                Distance._last_sensor_data = {
                    "front": Distance._unpack_distance(packet, 0),
                    "rear": Distance._unpack_distance(packet, 2),
                    "left": Distance._unpack_distance(packet, 4),
                    "right": Distance._unpack_distance(packet, 6),
                }
        except Exception as e:
            print(f"Sensor data parse error: {e}")

        return Distance._last_sensor_data.copy()

    @staticmethod
    def get_front() -> float:
        return Distance.get_sensor_data()["front"]

    @staticmethod
    def get_rear() -> float:
        return Distance.get_sensor_data()["rear"]

    @staticmethod
    def get_left() -> float:
        return Distance.get_sensor_data()["left"]

    @staticmethod
    def get_right() -> float:
        return Distance.get_sensor_data()["right"]


def get_sensor_data():
    """
    Backwards‑compatible function wrapper.

    Usage:
        from distance import get_sensor_data
        data = get_sensor_data()
    """
    return Distance.get_sensor_data()