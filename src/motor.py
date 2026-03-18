from machine import Pin, PWM

from state import state

class Motor:
    in1 = Pin(2, Pin.OUT)
    in2 = Pin(1, Pin.OUT)
    ena = PWM(Pin(0))
    ena.freq(1000)

    @staticmethod
    def frente():
        Motor.in1.on()
        Motor.in2.off()
        

    @staticmethod
    def tras():
        Motor.in1.off()
        Motor.in2.on()

    @staticmethod
    def parar():
        Motor.in1.off()
        Motor.in2.off()
        state.target_speed = 0.0
