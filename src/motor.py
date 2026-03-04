from machine import Pin, PWM
import time

class Motor:
    in1 = Pin(2, Pin.OUT)
    in2 = Pin(1, Pin.OUT)
    ena = PWM(Pin(0))
    ena.freq(1000)

    @staticmethod
    def frente(velocidade):
        Motor.in1.on()
        Motor.in2.off()
        Motor.ena.duty_u16(int(velocidade * 65535))

    @staticmethod
    def tras(velocidade):
        Motor.in1.off()
        Motor.in2.on()
        Motor.ena.duty_u16(int(velocidade * 65535))

    @staticmethod
    def parar():
        Motor.in1.off()
        Motor.in2.off()
        Motor.ena.duty_u16(0)


# Funções antigas mantidas por compatibilidade
def frente(velocidade):
    Motor.frente(velocidade)


def tras(velocidade):
    Motor.tras(velocidade)


def parar():
    Motor.parar()
