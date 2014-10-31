import sys
from subprocess import call
import time

MOTOR_L = 0x0a
MOTOR_R = 0x0b

def i2cwrite(address, register, value):
    print address, register, value
    call(["i2cset", "-y", "1", str(address), str(register), str(value)])

def i2cwritel(address, register, value):
    print address, register, value
    call(["i2cset", "-y", "1",  str(address), str(register), str(value), "w"])

def init_motor(motor):
    i2cwrite(motor, 0x01, 0x13)

def set_speed(motor, speed):
    if (speed < 0):
        speed = hex(((abs(speed) ^ 0xffff) + 1) & 0xffff)
    i2cwritel(motor, 0x04, 0x0000)
    i2cwritel(motor, 0x06, speed)
    i2cwritel(motor, 0x90, 0x0004)
    i2cwritel(motor, 0x80, 0xffff)
    i2cwrite(motor, 0x08, 0x00)

class Chassis:
    def __init__(self, motor_left, motor_right):
        self.motor_left = motor_left
        self.motor_right = motor_right
        init_motor(motor_left)
        init_motor(motor_right)

    def move(self, speed_left, speed_right):
        set_speed(self.motor_left, speed_left)
        set_speed(self.motor_right, speed_right)

    def stop(self):
        self.move(0,0)

chassis = Chassis(MOTOR_L, MOTOR_R)
chassis.move(-100,100)
time.sleep(1.0)
chassis.stop()
