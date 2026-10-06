from machine import freq

freq(int(3 * 100_000_000))

import _thread

from servo import Servo
from uart import Uart
from camera import Camera
from background import background_task


Uart.initialize()
Camera.initialize(uart_id=1, rx_pin=9)
_thread.start_new_thread(background_task, ())
Servo.set_angle(0)
