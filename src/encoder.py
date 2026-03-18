from machine import Pin
import time
import math


class Encoder:
    # ===== CONFIGURAÇÃO =====
    PPR = 200               # pulsos por volta do encoder
    RELACAO = 3 / 5         # relação engrenagens
    DIAMETRO_RODA = 3.2     # centimetros

    # Estado
    contador = 0

    # Pinos (estáticos)
    encoder_a = Pin(7, Pin.IN, Pin.PULL_UP)
    encoder_b = Pin(6, Pin.IN, Pin.PULL_UP)

    @staticmethod
    def encoder_irq(pino):
        if Encoder.encoder_b.value() == 0:
            Encoder.contador += 1   # Frente
        else:
            Encoder.contador -= 1   # Trás

    @staticmethod
    def setup_irq():
        # Interrupção na borda de subida do canal A
        Encoder.encoder_a.irq(trigger=Pin.IRQ_RISING, handler=Encoder.encoder_irq)

    @staticmethod
    def distance_cm():
        return Encoder.contador / (Encoder.PPR / Encoder.RELACAO) * (Encoder.DIAMETRO_RODA * math.pi)

# Configura interrupção e inicia o loop principal
Encoder.setup_irq()