from uart import Uart
from distance import Distance
import time


def main():
    # Initialize UART for distance sensors
    Uart.initialize()

    print("Starting sonar distance readings (cm)...")

    while True:
        data = Distance.get_sensor_data()

        # Data keys: front, rear, left, right
        print(
            "Front: {front:5.1f}  Rear: {rear:5.1f}  "
            "Left: {left:5.1f}  Right: {right:5.1f}".format(**data)
        )

        # Adjust delay as needed (in milliseconds)
        time.sleep_ms(100)


if __name__ == "__main__":
    main()

