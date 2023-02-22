import sys
import numpy as np
import cv2
sys.path.append('../src')
from robot_simulation import Robot


def main():
    # create the instance of the
    # robor
    robot = Robot('../maps/map_1.png', top_view_zoom=4)

    while True:
        # get the image
        img = robot.get_camera_view()

        # convert it to gray
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # apply a threshold
        ret, gray = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        # count the black pixels to the right and to the left
        black_pixels_dx = np.count_nonzero(255-gray[:, 32:])
        black_pixels_sx = np.count_nonzero(255 - gray[:, :32])

        delta = black_pixels_dx-black_pixels_sx

        print(delta)

        v_dx = 100 - delta/10,
        v_sx = 100 + delta/10

        v_dx = np.clip(v_dx, -255, 255)
        v_sx = np.clip(v_sx, -255, 255)

        robot.set_motors_speeds(
            v_dx,
            v_sx
        )

        cv2.imshow("view", cv2.resize(gray, (500, 500)))
        cv2.waitKey(1)

    # delete the robot, to stop the treads
    robot.__del__()


if __name__ == '__main__':
    main()
