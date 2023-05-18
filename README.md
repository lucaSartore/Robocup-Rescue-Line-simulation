# RescueLine simulation

this is a simulation of a robot with a camera for the RoboCup recue line competition

you can use this to speed up the development of your robot, and/or start coding before the robot is completed


the program that me and my them used to won first place in the world championship was 90% written in this simulation

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/WPVMSKnJaNo/0.jpg)](https://www.youtube.com/watch?v=WPVMSKnJaNo)

## How it looks

here is a video example of how to expect from the simulation

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/lHfZ9TxOEeg/0.jpg)](https://www.youtube.com/watch?v=lHfZ9TxOEeg)

## How to use it

you first need to install opencv and numpy by running the following commands:

```
pip install opencv-python
pip install numpy
```

then just clone the repository and import the `Robot` class from the `robot_simulation.py` file

then you can use the built-in functions to develop the program, see the [example_1.py](examples/example_1.py) file

## Customization

the robot is highlight customizable to better fit your needs...
you can customize things like the width and speed of the robot, the dimension and position of the camera and many more.

do customize your robot yuo need to pass the respective parameter to the robot constructor, the documentation is in the code [here](https://github.com/lucaSartore/Robocup-Rescue-Line-simulation/blob/main/src/robot_simulation.py#L165)

## Explanation of the line follower software

if you want more details on how our original program work, you can find a full explanation [here](https://github.com/lucaSartore/Robocup-Rescue-Line-simulation/blob/main/line_follower_program.pdf)
