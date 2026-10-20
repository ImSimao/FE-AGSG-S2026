from machine import Pin, PWM

from state import state

class Motor:
    in1 = Pin(2, Pin.OUT)
    in2 = Pin(1, Pin.OUT)
    ena = PWM(Pin(0))
    ena.freq(20000)  # 20 kHz: switching noise above the encoder band (easier to filter)

    @staticmethod
    def forward():
        Motor.in1.on()
        Motor.in2.off()
        

    @staticmethod
    def backward():
        Motor.in1.off()
        Motor.in2.on()

    @staticmethod
    def stop():
        Motor.in1.off()
        Motor.in2.off()
        state.target_speed = 0.0
