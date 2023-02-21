import cv2
# this file has the main class that allows you to simulate your robot

# the default dimension in centimeter that the camera see horizontally
DEFAULT_CAMERA_X_DIMENSION: float = 10.

# the default dimension in centimeter that the camera see vertically
DEFAULT_CAMERA_Y_DIMENSION: float = 10.

# the default PPI (pixel per inch) of the image that get read
DEFAULT_PPI: int = 72


class Robot:
    def __init__(self, map_path: str):
        self.pos_x: float = 0
        self.pos_y: float = 0
        self.angle: float = 0
        self.map = cv2.imread(map_path)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f"Robot at X:{self.pos_x} Y:{self.pos_y} angle:{self.angle}"


def test():

    r = Robot("../maps/map_1.png")


test()