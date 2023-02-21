import cv2
# this file has the main class that allows you to simulate your robot

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

        self.pos_x: float = start_pos_x
        self.pos_y: float = start_pos_y
        self.angle: float = start_angle
        self.max_speed: float = _max_speed
        self.ppi: float = _ppi
        self.map = cv2.imread(map_path)
        self.camera_x_dimension: float = _camera_x_dimension
        self.camera_y_dimension: float = _camera_y_dimension
        self.camera_x_offset: float = _camera_x_offset
        self.robot_wight: float = _robot_wight

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f"Robot at X:{self.pos_x} Y:{self.pos_y} angle:{self.angle}"


def test():

    r = Robot("../maps/map_1.png")


test()