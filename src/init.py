from machine import freq

freq(int(3 * 100_000_000))

from servo import Servo
from uart import Uart
from camera import Camera


Uart.initialize()
Camera.initialize(uart_id=1, rx_pin=9)
Servo.set_angle(0)
