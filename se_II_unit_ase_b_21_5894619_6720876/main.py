#!/usr/bin/env pybricks-micropython
from pybricks import ev3brick as brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase


left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
wheel_diameter = 56
axle_track = 114

robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

color_Sensor = ColorSensor(Port.S1)

def motor_start():
    robot.drive(100,0)

def motor_start():
    robot.stop()

def scan():
    while Color.RED != color_Sensor.color():
        wait(10)
        
def turnaround():
    robot.drive_time(-100, 0, 2000)
    robot.drive_time(0, 60, 3000)


while True:
    motor_start()
    scan()
    motor_stop()
    turnaround()
