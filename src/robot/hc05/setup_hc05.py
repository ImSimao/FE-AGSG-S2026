import machine
import time

# Wiring: Pico TX0=GP0, RX0=GP1. Swap if your wiring differs.
# Enter AT mode: hold KEY/EN high (or press+hold side button) before powering HC-05.

uart = machine.UART(
    0,
    baudrate=38400,           # default AT baud for HC-05
    tx=machine.Pin(0),
    rx=machine.Pin(1),
    timeout=2000,             # ms
)


def send_at(cmd, wait=0.5):
    uart.write(cmd + "\r\n")
    time.sleep(wait)
    resp = uart.read()
    print(f"{cmd!r} -> {resp}")
    return resp


# Clear any stale data
uart.read()

send_at("AT", wait=0.8)                # expect b'OK'
send_at("AT+VERSION?", wait=0.8)
send_at("AT+UART?", wait=0.8)

send_at("AT+NAME=WRO", wait=0.8)

# Some HC-05 firmwares need quoted PSWD. Try unquoted, then quoted if ERROR.
resp = send_at("AT+PSWD=1234", wait=0.8)
if resp and b"ERROR" in resp:
    send_at('AT+PSWD="1234"', wait=0.8)

send_at("AT+UART=115200,0,0", wait=0.8)

# Re-init to new baud and verify
uart.init(baudrate=115200, timeout=2000)
send_at("AT", wait=1.0)