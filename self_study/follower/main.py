#!/usr/bin/env pybricks-micropython
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                UltrasonicSensor, GyroSensor, UltrasonicSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase


left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
wheel_diameter = 56
axle_track = 114
max_distance = 400

robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
sensor = UltrasonicSensor(Port.S1)
gyroSensor = GyroSensor(Port.S3)
isDriving = False


speed = 130
max_angle = 3
adjustment_time = 350

def motor_start():
    robot.drive(speed,0)

def motor_stop():
    robot.stop()

def read_signal():
    global isDriving
    currentDistance = sensor.distance()
    if(currentDistance > max_distance):
        motor_start()
        isDriving = True
    else:
        motor_stop()
        isDriving = False

def adjust_direction():  
    global isDriving
    if(isDriving):
        curAngle = gyroSensor.angle()
        if (curAngle > max_angle):
            robot.drive_time(speed, -5, adjustment_time)
            brick.display.text("Adjusting")
        if (curAngle < -max_angle):
            robot.drive_time(speed, 5, adjustment_time)
            brick.display.text("Adjusting")
    

max_distance = sensor.distance()
gyroSensor.reset_angle(0)
while True:
    
    adjust_direction()
    read_signal()
    wait(10)
