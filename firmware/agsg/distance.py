from uart import Uart


class Distance:
    """
    Static helper for reading distance sensors over UART.

    Protocol (v2) — 10 bytes per packet:
        byte 0      : header 0xA5
        bytes 1..8  : 4 x uint16 little-endian (front, rear, left, right),
                      each value scaled by 10 (1 decimal place);
                      0xFFFF means "no reading" (mapped to -1.0)
        byte 9      : checksum = XOR of bytes 1..8

    The receiver buffers incoming bytes and re-synchronises on the header,
    so a dropped/garbage byte no longer causes permanent desync.
    """

    _HEADER = 0xA5
    _PACKET_LEN = 10  # header + 8 payload + 1 checksum
    _rx_buffer = b""

    # Cached last known distances (in cm)
    _last_sensor_data = {
        "front": -1.0,
        "rear": -1.0,
        "left": -1.0,
        "right": -1.0,
    }

    @staticmethod
    def _unpack_distance(payload, offset):
        """Unpack a 16-bit little-endian integer from the payload (in cm)."""
        low_byte = payload[offset]
        high_byte = payload[offset + 1]
        scaled_value = low_byte | (high_byte << 8)

        # Error value from sensor
        if scaled_value == 0xFFFF:
            return -1.0

        return scaled_value / 10.0

    @staticmethod
    def _read_packet():
        """
        Read UART, buffer bytes and decode one 8-byte payload.

        Re-synchronises on the header byte and validates the checksum.
        Returns the 8-byte payload, or None if no complete valid packet
        is available yet.
        """
        uart = getattr(Uart, "_uart", None)
        if uart is None:
            return None

        try:
            available = uart.any()
            if not available:
                return None

            data = uart.read(available)
            if not data:
                return None

            Distance._rx_buffer += data
            if len(Distance._rx_buffer) > 32:
                Distance._rx_buffer = Distance._rx_buffer[-32:]

            payload = None
            while True:
                idx = Distance._rx_buffer.find(bytes([Distance._HEADER]))
                if idx < 0:
                    # No header: keep only the tail that could start a packet.
                    Distance._rx_buffer = Distance._rx_buffer[-(Distance._PACKET_LEN - 1):]
                    break

                if idx > 0:
                    # Discard garbage before the header.
                    Distance._rx_buffer = Distance._rx_buffer[idx:]

                if len(Distance._rx_buffer) < Distance._PACKET_LEN:
                    # Incomplete packet: wait for more bytes.
                    break

                body = Distance._rx_buffer[1:9]
                checksum = Distance._rx_buffer[9]

                calc = 0
                for b in body:
                    calc ^= b

                if calc == checksum:
                    payload = body
                    Distance._rx_buffer = Distance._rx_buffer[Distance._PACKET_LEN:]
                    # Keep scanning in case a newer packet is buffered.
                else:
                    # Bad checksum: skip this header and keep searching.
                    Distance._rx_buffer = Distance._rx_buffer[1:]

            return payload
        except Exception as e:
            print("UART read error (distance):", e)
            return None

    @staticmethod
    def get_sensor_data():
        """
        Read one fresh packet and update the cache. Returns last known values.
        """
        try:
            payload = Distance._read_packet()
            if payload is not None:
                Distance._last_sensor_data = {
                    "front": Distance._unpack_distance(payload, 0),
                    "rear": Distance._unpack_distance(payload, 2),
                    "left": Distance._unpack_distance(payload, 4),
                    "right": Distance._unpack_distance(payload, 6),
                }
        except Exception as e:
            print("Sensor data parse error:", e)

        return Distance._last_sensor_data.copy()

    @staticmethod
    def get_front():
        return Distance._last_sensor_data["front"]

    @staticmethod
    def get_rear():
        return Distance._last_sensor_data["rear"]

    @staticmethod
    def get_left():
        return Distance._last_sensor_data["left"]

    @staticmethod
    def get_right():
        return Distance._last_sensor_data["right"]


def get_sensor_data():
    """
    Backwards-compatible function wrapper.
    """
    return Distance.get_sensor_data()
