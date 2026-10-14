from machine import Pin
import time
import math


# ===== CONFIGURAÇÃO =====
PPR = 200               # pulsos por volta do encoder
RELACAO = 3 / 5         # relação engrenagens
DIAMETRO_RODA = 3.2     # centimetros

# Debounce dinâmico (ajustado pela velocidade):
MIN_DEBOUNCE_US = 20    # alta velocidade (pulsos próximos)
MAX_DEBOUNCE_US = 1500  # baixa velocidade (pulsos espaçados)

# Pinos (estáticos)
_encoder_a = Pin(7, Pin.IN, Pin.PULL_UP)
_encoder_b = Pin(6, Pin.IN, Pin.PULL_UP)

# Estado — globais de módulo para acesso rápido dentro do IRQ
_contador = 0
_last_edge_us = 0
_debounce_us = 200  # valor inicial


def update_debounce_from_speed(speed_cm_s):
    """Ajusta o debounce ao período esperado do encoder (1/4 do período)."""
    global _debounce_us
    if speed_cm_s <= 0.5:
        _debounce_us = MAX_DEBOUNCE_US
        return
    # período esperado (µs) = 1e6 / (contagens/s)
    # contagens/s = velocidade * (PPR/RELACAO) / perímetro
    # => período ≈ 30158 / velocidade(cm/s)
    period_us = 30158.0 / speed_cm_s
    debounce = int(period_us / 4)
    if debounce < MIN_DEBOUNCE_US:
        debounce = MIN_DEBOUNCE_US
    elif debounce > MAX_DEBOUNCE_US:
        debounce = MAX_DEBOUNCE_US
    _debounce_us = debounce


def _irq(pin):
    global _contador, _last_edge_us
    now = time.ticks_us()
    if time.ticks_diff(now, _last_edge_us) < _debounce_us:
        return  # bounce/ruído, ignora
    _last_edge_us = now
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
