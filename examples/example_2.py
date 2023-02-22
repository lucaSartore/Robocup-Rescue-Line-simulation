"""
this example has been created with chat gpt, using the following prompt

I need you to create a program for a robot that follows a line, using python, opencv and numpy.

The line is black on a white background and is approximately 2 cm wide. The robot has a camera in front that can see a 10 cm x 10 cm portion of the floor.

the robot is on track, and you can set the speed using the function robot.set_motors_speeds(speed_right_motor,speed_left_motor)

the speed bust be in the -255 to 255 range

you can get the image from the camera using: img = robot.get_camera_view().

with this you will receive an array 64x64x3 where the first dimension is the Y and goes top to bottom, the second dimension is the X and goes left to right, and the third dimension contains the colors in the B G R order

can you write this python program for me?


then i also hask him:

can you add some sort of "visualization" to the program?
meybe a windows that cereate a circle arround cx and cy so that you can see what you are following

"""

import sys
import numpy as np
import cv2
sys.path.append('../src')
from robot_simulation import Robot


lower_black = np.array([0, 0, 0])
upper_black = np.array([179, 255, 50])

def main():
    # create the instance of the
    # robor
    robot = Robot('../maps/map_1.png', top_view_zoom=4)

    while True:
        # Get the camera view
        img = robot.get_camera_view()

        # Convert the image to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Threshold the image to get only the black line
        mask = cv2.inRange(hsv, lower_black, upper_black)

        # Find the contours of the black line
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # If there is a contour, follow the line
        if len(contours) > 0:
            # Get the largest contour
            largest_contour = max(contours, key=cv2.contourArea)

            # Get the centroid of the contour
            M = cv2.moments(largest_contour)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # Draw a circle around the centroid
            img = cv2.circle(img, (cx, cy), 5, (0, 255, 0), -1)

            # Calculate the error from the center of the image
            error = cx - 32

            # Calculate the motor speeds
            speed = 100
            speed_left = speed + error
            speed_right = speed - error

            # Set the motor speeds
            robot.set_motors_speeds(speed_right, speed_left)

        # Display the image with the circle
        cv2.imshow('Line Follower', cv2.resize(img, (500, 500)))
        cv2.waitKey(1)
    # delete the robot, to stop the treads
    robot.__del__()




if __name__ == '__main__':
    main()
