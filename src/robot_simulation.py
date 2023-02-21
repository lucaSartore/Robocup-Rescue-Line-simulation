import cv2
from dataclasses import dataclass

# this file has the main class that allows you to simulate your robot


class Robot:
    def __init__(self, map_path: str):
        self.pos_x: float = 0
        self.pos_y: float = 0
        self.angle: float = 0

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f"Robot at X:{self.pos_x} Y:{self.pos_y} angle:{self.angle}"


def test():

    r = Robot()

    print(r)

test()