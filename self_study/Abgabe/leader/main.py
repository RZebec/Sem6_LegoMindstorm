#!/usr/bin/env pybricks-micropython
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor, InfraredSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase

# Configuration, which depends on the build of the roboter:
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
wheel_diameter = 56
axle_track = 114
ir_sensor = InfraredSensor(Port.S4)
gyroSensor = GyroSensor(Port.S3)

# Configuration regarding the behavior of the robot:
# speed of the robot:
speed = 130
# maximum deviation in the drive-direction:
max_angle = 3
# will be dynamically set during the application start:
adjustment_time = 350

# Global variables:
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
isDriving = False

# Start the motor and let the robot drive:
def motor_start():
    robot.drive(speed,0)

# Stop the motor and the robot:
def motor_stop():
    robot.stop()

# Read the gyro sensor and adjust the direction of the robot:
def adjust_direction():  
    # Only adjust the direction when the robot is moving:
    global isDriving
    if(isDriving):
         # Get the current angle and compare it with the initial angle:
        curAngle = gyroSensor.angle()
        if (curAngle > max_angle):
            robot.drive_time(speed, -5, adjustment_time)
            motor_start()
            brick.display.text("Adjusting")
        if (curAngle < -max_angle):
            robot.drive_time(speed, 5, adjustment_time)
            brick.display.text("Adjusting")
            motor_start()

# Read the infrared sensor to react to button presses:
def read_ir_signal():
    global isDriving
    listOfPressedButtons = ir_sensor.buttons(1)
    print(listOfPressedButtons)
    if (Button.LEFT_UP in listOfPressedButtons):
        brick.display.text("UP")
        motor_start()
        isDriving = True
        return
    if(Button.LEFT_DOWN in listOfPressedButtons):
        brick.display.text("DOWN")
        motor_stop()
        isDriving = False
        return

# The main loop which orchestrates the different functions:
while True:
    read_ir_signal()
    adjust_direction()
    wait(10)