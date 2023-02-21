# this file has the main class that allows you to simulate your robot

import cv2
import time
from threading import Thread
import numpy as np
import sys
# constant for the "shape" of the image
from image_shape_definition import X, Y, C, BLUE, GREEN, RED, ALPHA

from util import copy_and_paste_image

# the default dimension in centimeter that the camera see horizontally
DEFAULT_CAMERA_X_DIMENSION: float = 10.

# the default dimension in centimeter that the camera see vertically
DEFAULT_CAMERA_Y_DIMENSION: float = 10.

# the centimeters the camera is offset from the center of the robot
DEFAULT_CAMERA_X_OFFSET: float = 5.

# the default wight dimension of the robot in centimeter
DEFAULT_ROBOT_WIGHT: float = 17

# the default PPI (pixel per inch) of the image that get read
DEFAULT_PPI: int = 17

# the default maximum speed of the robot (in cm/s)
DEFAULT_MAX_SPEED: float = 10.

# the default starting position of the robot presented in cm
DEFAULT_START_POS_X: float = 0.

# the default starting position of the robot presented in cm
DEFAULT_START_POS_Y: float = 0.

# the default starting angle of the robot presented in radiant
DEFAULT_START_ANGLE: float = 0.

# the resolution of the top view
DEFAULT_TOP_VIEW_RES_X: int = 1100
# the resolution of the top view
DEFAULT_TOP_VIEW_RES_Y: int = 700

# whether the top vew is enabled or not
DEFAULT_TOP_VIEW_ENABLE: bool = True

# the eventual zoom of the top view
DEFAULT_TOP_VIEW_ZOOM: float = 1.

# the time step of the simulation, the lower it is the more precise the simulation is
DEFAULT_SIMULATION_TIME_STEP: float = 0.001


# sum to an x value a vector and return the new x vaue
def __vec_sum_x(x: int, angle: float, module: float) -> int:
    return int(x + np.cos(angle)*module)


# sum to an y value a vector and return the new x vaue
def __vec_sum_y(y: int, angle: float, module: float) -> int:
    return int(y + np.sin(angle)*module)


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
                 robot_wight: float = DEFAULT_ROBOT_WIGHT,
                 top_view_res_x: float = DEFAULT_TOP_VIEW_RES_X,
                 top_view_res_y: float = DEFAULT_TOP_VIEW_RES_Y,
                 top_view_enable: bool = DEFAULT_TOP_VIEW_ENABLE,
                 top_view_zoom: float = DEFAULT_TOP_VIEW_ZOOM,
                 simulation_time_step: float = DEFAULT_SIMULATION_TIME_STEP
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
        # the image representing the robot itself
        self.__robot_image = self.__get_robot_image()
        # resolution of the rop view
        self.__top_view_res_x = top_view_res_x
        # resolution of the rop view
        self.__top_view_res_y = top_view_res_y
        # the zoom of the top view
        self.__top_view_zoom = top_view_zoom
        # the time step of the simulation
        self.__simulation_time_step = simulation_time_step

        # launch the top view
        if top_view_enable:
            self.__top_view_thread = Thread(target=self.__update_top_view, args=())
            self.__top_view_thread.start()
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

        min_pos_x = 0
        min_pos_y = 0

        max_pos_x = self.__pixel_to_cm(self.__map.shape[X])
        max_pos_y = self.__pixel_to_cm(self.__map.shape[Y])

        time_sample = time.time()

        while self.__thread_running:

            self.__pos_x += self.__speed_left/100
            self.__pos_y += self.__speed_right/100

            # keep the robot inside the map
            self.__pos_x = np.clip(self.__pos_x, min_pos_x, max_pos_x)
            self.__pos_y = np.clip(self.__pos_y, min_pos_y, max_pos_y)

            time_to_wait = self.__simulation_time_step - (time.time()-time_sample)
            if time_to_wait < 0:
                print("Lack of precision due to CPU too slow... try increase the simulation time step", file=sys.stderr)
            else:
                time.sleep(time_to_wait)
            time_sample = time.time()

    #sum to a x value a vector

    # update the top view
    def __update_top_view(self):
        while self.__thread_running:
            robot = self.__get_robot_image()
            image_center = tuple(np.array(robot.shape[1::-1]) / 2)
            rot_mat = cv2.getRotationMatrix2D(image_center, self.__angle*180/np.pi, 1.0)
            robot = cv2.warpAffine(robot, rot_mat, robot.shape[1::-1], flags=cv2.INTER_LINEAR)

            # put the robot inside the map
            img = copy_and_paste_image(
                paste_to_=self.__map,
                copy_from_=robot,
                middle_point_x=self.__cm_to_pixel(self.__pos_x),
                middle_point_y=self.__cm_to_pixel(self.__pos_y),
            )

            # eventual zoom the image
            if self.__top_view_zoom > 1:
                new_res_x = self.__map.shape[X]/self.__top_view_zoom
                new_res_y = self.__map.shape[Y]/self.__top_view_zoom

                center_x = self.__cm_to_pixel(self.__pos_x)
                center_y = self.__cm_to_pixel(self.__pos_y)

                # the new coordinates of the new map
                x1 = int(center_x-new_res_x/2)
                y1 = int(center_y-new_res_y/2)
                x2 = int(center_x + new_res_x / 2)
                y2 = int(center_y + new_res_y / 2)

                # be careful not to overflow
                x1 = np.clip(x1, 0, img.shape[X])
                x2 = np.clip(x2, 0, img.shape[X])
                y1 = np.clip(y1, 0, img.shape[Y])
                y2 = np.clip(y2, 0, img.shape[Y])

                # zoom the image
                img = img[y1:y2, x1:x2, :]

            # resize the image according to user inputs
            og_res_x = img.shape[X]
            og_res_y = img.shape[Y]

            if og_res_x/self.__top_view_res_x > og_res_y/self.__top_view_res_y:
                # res x is ok, scale res y
                new_res_x = self.__top_view_res_x
                new_res_y = int(self.__top_view_res_x/og_res_x*og_res_y)
            else:
                # res y is ok, scale res x
                new_res_y = self.__top_view_res_y
                new_res_x = int(self.__top_view_res_y / og_res_y * og_res_x)

            img = cv2.resize(img, (new_res_x, new_res_y))

            cv2.imshow("TOP VIEW use Robot(..., top_view_enable = False) to disable", img)
            cv2.waitKey(1)
            time.sleep(0.05)

    # set the speed of the motors
    def set_motors_speeds(self, right: int, left: int):
        assert -255 <= right <= 255 and -255 <= left <= 255, "the speeds of the motor MUST be in the -255 to 255 range"
        self.__speed_right = right
        self.__speed_left = left

    # return an image that represent the robot for the visualization
    def __get_robot_image(self):

        # find the max dimension possible
        x_max = float(np.abs(self.__camera_x_offset)) + self.__camera_x_dimension/2
        y_max = self.__robot_wight/2

        # find the biggest dimension
        dimension_max = x_max
        if y_max > dimension_max:
            dimension_max = y_max

        # find rhe final dimension with a 1.5 safety margin
        dimension = self.__cm_to_pixel(dimension_max*2*np.sqrt(2)*1.5)

        # create the image
        image = np.zeros(shape=[dimension, dimension, 4], dtype=np.uint8)

        # load the body
        body = cv2.imread("../images/body.png", cv2.IMREAD_UNCHANGED)
        body_dim = self.__cm_to_pixel(self.__robot_wight*0.7)
        body = cv2.resize(body, (body_dim, body_dim))

        # load the track
        track = cv2.imread("../images/track.png", cv2.IMREAD_UNCHANGED)
        track_y = int(body_dim * 1.3)
        track_x = self.__cm_to_pixel(3)
        track = cv2.resize(track, (track_x,track_y))

        # calculate the center of the image
        center_x = int(image.shape[X]/2)
        center_y = int(image.shape[Y]/2)

        # put the track in
        image = copy_and_paste_image(image, track, center_x - self.__cm_to_pixel(self.__robot_wight/2*0.8), center_y)
        # put the track in
        image = copy_and_paste_image(image, track, center_x + self.__cm_to_pixel(self.__robot_wight / 2*0.8), center_y)

        # put the body in
        image = copy_and_paste_image(image, body, center_x, center_y)

        # the coordinates of the camera view
        camera_x_center = center_x - self.__cm_to_pixel(self.__camera_x_offset)
        camera_y_center = center_y

        camera_delta_x = self.__cm_to_pixel(self.__camera_x_dimension/2)
        camera_delta_y = self.__cm_to_pixel(self.__camera_y_dimension / 2)

        camera_x1 = camera_x_center - camera_delta_x
        camera_y1 = camera_y_center - camera_delta_y

        camera_x2 = camera_x_center + camera_delta_x
        camera_y2 = camera_y_center + camera_delta_y

        # insert the square of the camera
        image = cv2.rectangle(image, (camera_y1,camera_x1), (camera_y2,camera_x2), (255, 0, 0,255), 4)

        # insert the transparency

        temp = image[:, :, ALPHA].copy()

        temp = cv2.rectangle(temp, (camera_y1, camera_x1), (camera_y2, camera_x2), 0,-1)

        image[:, :, ALPHA] = temp

        return image

    def __pixel_to_cm(self, pixel: int) -> float:
        inches = float(pixel)/self.__ppi
        cm = inches * 2.54
        return cm

    def __cm_to_pixel(self, cm: float) -> int:
        inches = cm / 2.54
        pixel = int(inches * self.__ppi)
        return pixel


def test():

    r = Robot("../maps/map_1.png", top_view_zoom=3, start_angle=-3.1415/2)

    # image = r.get_robot_image()
    #
    #
    #
    # mapp = cv2.imread("../maps/map_1.png")
    #
    # mapp = mapp[:, :, 0:3]
    #
    # print("out: ", mapp.shape)
    #
    # new = copy_and_paste_image(mapp,image,500,500)
    #
    #
    # cv2.imshow("test", new)
    # cv2.waitKey(0)

    r.set_motors_speeds(100, 236)

    for i in range(1000):
        print(r)
        time.sleep(0.2)

    r.__del__()



    # track = cv2.imread("../images/track.png", cv2.IMREAD_UNCHANGED)
    #
    # print(np.shape(track))
    #
    # cv2.imshow("test", track[:,:,3])
    # cv2.waitKey(1000000000)


test()
