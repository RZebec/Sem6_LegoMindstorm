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
ir_sensor = InfraredSensor(Port.S4)
gyroSensor = GyroSensor(Port.S3)


speed = 130
max_angle = 3
adjustment_time = 350

robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

isDriving = False


def motor_start():
    robot.drive(speed,0)

def motor_stop():
    robot.stop()

def adjust_direction():  
    if(isDriving):
        curAngle = gyroSensor.angle()
        if (curAngle > max_angle):
            robot.drive_time(speed, -5, adjustment_time)
            motor_start()
            brick.display.text("Adjusting")
        if (curAngle < -max_angle):
            robot.drive_time(speed, 5, adjustment_time)
            brick.display.text("Adjusting")
            motor_start()

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

while True:
    read_ir_signal()
    print(isDriving)
    adjust_direction()
    wait(10)