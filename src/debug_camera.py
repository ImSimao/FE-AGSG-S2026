import time
from machine import freq

freq(int(2.5 * 100_000_000))

from camera import Camera, get_traffic_lane_inside_pov
from field import Field
from state import state

# Camera.update() only reads UART when clockwise != 0
state.clockwise = 1
state.current_lane = 0
state.compass_angle = 0.0
state.set_relative_odom(150, 60)

Camera.initialize(uart_id=1, rx_pin=9)

print("Debug camera iniciado")
print("Posicao simulada: x=150 y=60 | clockwise=1")

last_key = None
heartbeat = 0

while True:
    blobs = Camera.get_blobs()
    pov = get_traffic_lane_inside_pov()
    zones = Field.LANES[state.relative_lane].zones
    key = (tuple(tuple(b) for b in blobs), tuple(tuple(z) for z in zones))

    heartbeat += 1
    if key != last_key or heartbeat >= 20:
        heartbeat = 0
        last_key = key

        if blobs:
            for cx, cy, color in blobs:
                print("Blob: cx={} cy={} {}".format(cx, cy, color))
        else:
            print("Sem detecoes")

        if pov:
            for p in pov:
                print("  POV pos={} lado={} ang={:.1f} dist={:.1f}".format(
                    p["pos"], p["side"], p["angle"], p["distance"]))

        print("Zonas:", zones)
        print("---")

    time.sleep_ms(1000)
