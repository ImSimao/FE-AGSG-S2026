from move_coordinates import move_coordinates
from state import state

def test_pid():
    state.clockwise = -1
    move_coordinates(250, 0)
