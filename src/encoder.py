from machine import Pin
import math


# ===== CONFIGURAÇÃO =====
PPR = 200               # pulsos por volta do encoder
RELACAO = 3 / 5         # relação engrenagens
DIAMETRO_RODA = 3.2     # centimetros

# Pinos (estáticos)
_encoder_a = Pin(7, Pin.IN, Pin.PULL_UP)
_encoder_b = Pin(6, Pin.IN, Pin.PULL_UP)

# Estado — global de módulo para acesso rápido dentro do IRQ
_contador = 0


def _irq(pin):
    global _contador
    # Na borda de subida do canal A, o nível de B indica o sentido.
    if _encoder_b.value():
        _contador -= 1   # Trás
    else:
        _contador += 1   # Frente


# Interrupção na borda de subida do canal A
_encoder_a.irq(trigger=Pin.IRQ_RISING, handler=_irq)


class Encoder:
    """Wrapper que mantém a API antiga (Encoder.distance_cm())."""

    @staticmethod
    def distance_cm():
        return _contador / (PPR / RELACAO) * (DIAMETRO_RODA * math.pi)
