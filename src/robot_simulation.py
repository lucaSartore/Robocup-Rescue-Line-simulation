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
                 max_speed: float = DEFAULT_MAX_SPEED,
                 ppi: int = DEFAULT_PPI,
                 camera_x_dimension: float = DEFAULT_CAMERA_X_DIMENSION,
                 camera_y_dimension: float = DEFAULT_CAMERA_Y_DIMENSION,
                 camera_x_offset: float = DEFAULT_CAMERA_X_OFFSET,
                 robot_wight: float = DEFAULT_ROBOT_WIGHT
                 ):

        # x position in cm of the robot
        self.__pos_x: float = start_pos_x
        # y position in cm of the robot
        self.__pos_y: float = start_pos_y
        # x angle of the robot orientation (in radians)
        self.__angle: float = start_angle
        # maximum speed of the robot in cm/s
        self.__max_speed: float = max_speed
        # pixel per inch of the map the robot has memorized
        self.__ppi: float = ppi
        # the map the robot has memorized
        self.__map = cv2.imread(map_path)
        # the dimension in cm of the horizontal camera view
        self.__camera_x_dimension: float = camera_x_dimension
        # the dimension in cm of the vertical camera view
        self.__camera_y_dimension: float = camera_y_dimension
        # the offset the camera has from the center of the robot in cm
        self.__camera_x_offset: float = camera_x_offset
        # the wight of the robot
        self.__robot_wight: float = robot_wight
        # relative speed (from -255 to 255) of the left motor
        self.__speed_left: int = 0
        # relative speed (from -255 to 255) of the right motor
        self.__speed_right: int = 0
        # a boolean flag used to stopp the updater thread
        self.__thread_running: bool = True
        # launching the thread that update the position
        self.__updater_thread = Thread(target=self.__position_updater, args=())
        self.__updater_thread.start()

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f"Robot at X:{self.__pos_x} Y:{self.__pos_y} angle:{self.__angle}"

    # disable the thread when finish using it
    def __del__(self):
        self.__thread_running = False

    # the thread that constantly update the position of the robot
    def __position_updater(self):
        while self.__thread_running:

            self.__pos_x += self.__speed_left
            self.__pos_y += self.__speed_right

            time.sleep(0.1)

    def set_motors_speeds(self, right: int, left: int):
        assert -255 <= right <= 255 and -255 <= left <= 255, "the speeds of the motor MUST be in the -255 to 255 range"
        self.__speed_right = right
        self.__speed_left = left


def test():

    r = Robot("../maps/map_1.png")

    r.set_motors_speeds(100, 236)

    for i in range(10):
        print(r)
        time.sleep(0.2)


    r.__del__()
test()


