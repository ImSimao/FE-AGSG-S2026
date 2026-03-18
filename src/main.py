import init
import time
import move_coordinates


def main():
    move_coordinates.move_coordinates(200, 0)
    move_coordinates.move_coordinates(200, -200)
    move_coordinates.move_coordinates(0, -200)
    move_coordinates.move_coordinates(0, 0)


if __name__ == "__main__":
    main()

