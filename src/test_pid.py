from move_coordinates import move_coordinates
from state import state

async def test_pid():
    state.clockwise = -1
    await move_coordinates(250, 0)
