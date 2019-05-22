#!/usr/bin/env pybricks-micropython
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                UltrasonicSensor, GyroSensor, UltrasonicSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase


# Configuration, which depends on the build roboter:
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
sensor = UltrasonicSensor(Port.S1)
gyroSensor = GyroSensor(Port.S3)
wheel_diameter = 56
axle_track = 114

# Configuration regarding the behavior of the robot:
# speed of the robot:
speed = 130
# maximum deviation in the drive-direction:
max_angle = 3
# duration for the adjustment process:
adjustment_time = 350
# will be dynamically set during the application start
max_distance = 400

# Global variables:
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
isDriving = False

# Start the motor and let the robot drive:
def motor_start():
    robot.drive(speed,0)

# Stop the motor and the robot:
def motor_stop():
    robot.stop()

# Read the ultrasonic sensor to get the distance:
def read_signal():
    # global modifier, because we need to change the variable:
    global isDriving
    # Get the current distance and react:
    currentDistance = sensor.distance()
    if(currentDistance > max_distance):
        motor_start()
        isDriving = True
    else:
        motor_stop()
        isDriving = False

# Read the gyro sensor and adjust the direction of the robot:
def adjust_direction():  
    # Only adjust the direction when the robot is moving:
    global isDriving
    if(isDriving):
        # Get the current angle and compare it with the initial angle:
        curAngle = gyroSensor.angle()
        if (curAngle > max_angle):
            robot.drive_time(speed, -5, adjustment_time)
            brick.display.text("Adjusting")
        if (curAngle < -max_angle):
            robot.drive_time(speed, 5, adjustment_time)
            brick.display.text("Adjusting")
    
# The following values will be set during the start of the application:

# Get the current distance to the robot in front of the sensor:
# The value will be dynamically set during the application start:
max_distance = sensor.distance()

# Set the angle of the gyroscope to zero. The main loop will 
# evaluate the value and adjust the direction of the robot:
gyroSensor.reset_angle(0)

# The main loop which orchestrates the different functions:
while True:   
    # Adjust the direction in comparison to the angle of the gyroscope,
    # which is set during the start of the application: 
    adjust_direction()
    # Read the distance signal and start or stop the robot:
    read_signal()
    # Wait 10 milliseconds till the next iteration:
    wait(10)
