from machine import Pin, PWM

class Motor:
    in1 = Pin(2, Pin.OUT)
    in2 = Pin(1, Pin.OUT)
    ena = PWM(Pin(0))
    ena.freq(20000)  # 20 kHz: switching noise above the encoder band (easier to filter)

    @staticmethod
    def forward():
        Motor.in2.on()
        Motor.in1.off()

    @staticmethod
    def backward():
        Motor.in2.off()
        Motor.in1.on()

    @staticmethod
    def stop():
        Motor.in1.off()
        Motor.in2.off()
