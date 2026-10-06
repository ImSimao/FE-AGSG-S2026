import init
import uasyncio as asyncio
from obstacle_challenge import obstacle_challenge
from background import start_background_tasks
from state import state
from servo import Servo
from motor import Motor


async def main():
    start_background_tasks()
    try:
        await obstacle_challenge()
    finally:
        state.target_speed = 0
        Motor.parar()
        Motor.ena.duty_u16(0)
        Servo.set_angle(0)


if __name__ == "__main__":
    asyncio.run(main())
