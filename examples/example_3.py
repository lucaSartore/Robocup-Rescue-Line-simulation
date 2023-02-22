# this example is more complex, but also better ar traking the line


import sys
import numpy as np
import cv2
sys.path.append('../src')
from robot_simulation import Robot


def main():
    # create the instance of the
    # robot
    # windows
    robot = Robot('../maps/map_2.png')
    # linux
    # robot = Robot('../maps/map_2.png', top_view_enable=False)

    # create a mask that consider only the pixels around the border
    corner_mask = np.ones(shape=(64,64),dtype=np.uint8)*255
    corner_mask[10:, 10:-10] = 0

    # a mask that keep tracks of the latest position of the line
    previous_mask = np.ones(shape=(64, 64), dtype=np.uint8)*255

    # a black mask
    black = np.zeros(shape=(64, 64), dtype=np.uint8)

    while True:
        # Get the camera view
        img = robot.get_camera_view()

        # Threshold the image to get only the black line
        mask = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # mask the line in order to keep track only of the pace of the line close to the border
        mask[corner_mask == 0] = 255

        # a mask where the line is visible only if is close to the previous known position of the line
        mask_with_previous_position = mask.copy()
        mask_with_previous_position[previous_mask == 0] = 255



        # Find the contours of the black line
        contours, _ = cv2.findContours(255-mask_with_previous_position, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # If there is a contour, follow the line
        if len(contours) > 0:
            # Get the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            try:
                # Get the centroid of the contour
                M = cv2.moments(largest_contour)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # update the zone close to the previous position of the line
                previous_mask = black.copy()
                previous_mask = cv2.circle(previous_mask, (cx, cy), 10, 255, -1)
            except ZeroDivisionError:
                # if the robot loses the line it go back and reset the "previous_mask"
                previous_mask = black.copy() + 255
                robot.set_motors_speeds(-50, -50)
                continue

            # Draw a circle around the centroid
            img = cv2.circle(img, (cx, cy), 5, (0, 255, 0), -1)

            # Calculate the error from the center of the image
            error_x = cx - 32
            error_y = -(cy-20)

            # Calculate the motor speeds

            speed_left = error_y*5 + error_x*3
            speed_right = error_y*5 - error_x*3

            speed_left = np.clip(speed_left, -255, 255)
            speed_right = np.clip(speed_right, -255, 255)

            # Set the motor speeds
            robot.set_motors_speeds(speed_right, speed_left)
        else:
            # reset previous mask and go back
            robot.set_motors_speeds(-50, -50)
            previous_mask = black.copy() + 255
        # Display the image with the circle
        cv2.imshow('Line Follower', cv2.resize(img, (500, 500)))
        cv2.waitKey(1)

        # only in linux
        # robot.update_top_view()
    # delete the robot, to stop the treads
    robot.__del__()




if __name__ == '__main__':
    main()
