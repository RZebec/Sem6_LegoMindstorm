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
max_distance = 100

robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
sensor = UltrasonicSensor(Port.S1)


def motor_start():
    robot.drive(100,0)

def motor_stop():
    robot.stop()

def read_signal():
    currentDistance = sensor.distance()
    if(currentDistance > max_distance):
        motor_start()
    else:
        motor_stop()
    
while True:
    read_signal()
    wait(10)