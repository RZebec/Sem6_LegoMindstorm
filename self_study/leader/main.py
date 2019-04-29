#!/usr/bin/env pybricks-micropython
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor, InfraredSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase


left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
wheel_diameter = 56
axle_track = 114

robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
ir_sensor = InfraredSensor(Port.S1)


def motor_start():
    robot.drive(100,0)

def motor_stop():
    robot.stop()

def read_ir_signal():
    listOfPressedButtons = ir_sensor.buttons(1)
    if (Button.UP in listOfPressedButtons):
        motor_start()
        return
    if(Button.DOWN in listOfPressedButtons):
        motor_stop()
        return
while True:
    read_ir_signal()
    wait(10)