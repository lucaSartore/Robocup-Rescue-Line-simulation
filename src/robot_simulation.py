# this file has the main class that allows you to simulate your robot

import cv2
import time
from threading import Thread

# the default dimension in centimeter that the camera see horizontally
DEFAULT_CAMERA_X_DIMENSION: float = 10.

# the default dimension in centimeter that the camera see vertically
DEFAULT_CAMERA_Y_DIMENSION: float = 10.

# the centimeters the camera is offset from the center of the robot
DEFAULT_CAMERA_X_OFFSET: float = 10.

# the default wight dimension of the robot in centimeter
DEFAULT_ROBOT_WIGHT: float = 17

# the default PPI (pixel per inch) of the image that get read
DEFAULT_PPI: int = 72

# the default maximum speed of the robot (in cm/s)
DEFAULT_MAX_SPEED: float = 10.

# the default starting position of the robot presented in cm
DEFAULT_START_POS_X: float = 0.

# the default starting position of the robot presented in cm
DEFAULT_START_POS_Y: float = 0.

# the default starting angle of the robot presented in radiant
DEFAULT_START_ANGLE: float = 0.


class Robot:
    def __init__(self,
                 map_path: str,
                 start_pos_x: float = DEFAULT_START_POS_X,
                 start_pos_y: float = DEFAULT_START_POS_Y,
                 start_angle: float = DEFAULT_START_ANGLE,
                 _max_speed: float = DEFAULT_MAX_SPEED,
                 _ppi: int = DEFAULT_PPI,
                 _camera_x_dimension: float = DEFAULT_CAMERA_X_DIMENSION,
                 _camera_y_dimension: float = DEFAULT_CAMERA_Y_DIMENSION,
                 _camera_x_offset: float = DEFAULT_CAMERA_X_OFFSET,
                 _robot_wight: float = DEFAULT_ROBOT_WIGHT
                 ):

        # x position in cm of the robot
        self.pos_x: float = start_pos_x
        # y position in cm of the robot
        self.pos_y: float = start_pos_y
        # x angle of the robot orientation (in radians)
        self.angle: float = start_angle
        # maximum speed of the robot in cm/s
        self.max_speed: float = _max_speed
        # pixel per inch of the map the robot has memorized
        self.ppi: float = _ppi
        # the map the robot has memorized
        self.map = cv2.imread(map_path)
        # the dimension in cm of the horizontal camera view
        self.camera_x_dimension: float = _camera_x_dimension
        # the dimension in cm of the vertical camera view
        self.camera_y_dimension: float = _camera_y_dimension
        # the offset the camera has from the center of the robot in cm
        self.camera_x_offset: float = _camera_x_offset
        # the wight of the robot
        self.robot_wight: float = _robot_wight
        # relative speed (from -255 to 255) of the left motor
        self.speed_left: int = 0
        # relative speed (from -255 to 255) of the right motor
        self.speed_right: int = 0
        # a boolean flag used to stopp the updater thread
        self.thread_running: bool = True
        # launching the thread that update the position
        self.updater_thread = Thread(target=self.position_updater, args=())
        self.updater_thread.start()

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f"Robot at X:{self.pos_x} Y:{self.pos_y} angle:{self.angle}"

    # disable the thread when finish using it
    def __del__(self):
        self.thread_running = False

    # the thread that constantly update the position of the robot
    def position_updater(self):
        while self.thread_running:

            self.pos_x += self.speed_left
            self.pos_y += self.speed_right

            time.sleep(0.1)


def test():

    r = Robot("../maps/map_1.png")

    r.speed_right = 1
    r.speed_left = -2

    for i in range(10):
        print(r)
        time.sleep(0.2)

test()