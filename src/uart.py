from machine import UART, Pin


class Uart:
    """
    Static UART helper for board communication.

    This class exposes only class (static) methods and manages a single
    shared UART instance internally.
    """

    _uart = None

    @staticmethod
    def initialize(uart_id: int = 0, baudrate: int = 115200, tx_pin: int = 16, rx_pin: int = 17) -> None:
        """
        Initialize the global UART interface.

        Args:
            uart_id: UART interface ID.
            baudrate: Communication baud rate.
            tx_pin: TX pin number.
            rx_pin: RX pin number.
        """
        try:
            Uart._uart = UART(
                uart_id,
                baudrate=baudrate,
                tx=Pin(tx_pin),
                rx=Pin(rx_pin),
            )

            # Clear any existing data in the buffer
            Uart._uart.read()
        except Exception:
            Uart._uart = None
            raise
