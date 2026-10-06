import time
import uasyncio as asyncio
import move_coordinates
from distance import Distance
from state import state


async def open_challenge():

    state.clockwise = -1

    await move_coordinates.move_coordinates(50, 0)

    if Distance.get_front() > 50 or Distance.get_front() <= 0:
        await move_coordinates.move_coordinates(100, state.odom_y)

    await move_coordinates.rotate_angle(0)

    await asyncio.sleep(1)

    if Distance.get_left() < Distance.get_right() and (Distance.get_left() > 0 or Distance.get_right() <= 0):
        state.clockwise = 1
    else:
        state.clockwise = -1

    if state.clockwise == 1:
        state.set_relative_odom(300-Distance.get_front()-9, Distance.get_left()+3)
    else:
        state.set_relative_odom(300-Distance.get_front()-9, Distance.get_right()+3)

    state.current_lane = 1

    while state.current_lane < 12:
        await move_coordinates.move_coordinates(230, 60)
        state.current_lane += 1

    await move_coordinates.move_coordinates(150, 50)
