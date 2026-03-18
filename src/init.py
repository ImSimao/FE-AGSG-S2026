import _thread
from machine import freq
from servo import Servo
from uart import Uart
from background import background_task

freq(int(3 * 100_000_000))

Uart.initialize()
_thread.start_new_thread(background_task, ())
Servo.set_angle(0)