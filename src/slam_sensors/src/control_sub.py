#!/usr/bin/env python

import rospy, i2c, smbus
from std_msgs.msg import String
from robot import *

def init_motor(bus, motor):
    i2c.write(bus, motor, 0x01, 0x13)

def set_speed(bus, motor, speed):
    if (speed < 0):
        speed = ((abs(speed) ^ 0xffff) + 1) & 0xffff
    i2c.writel(bus, motor, 0x04, 0x0000)
    i2c.writel(bus, motor, 0x06, speed)
    i2c.writel(bus, motor, 0x90, 0x0004)
    i2c.writel(bus, motor, 0x80, 0xffff)
    i2c.write(bus, motor, 0x08, 0x00)

class Chassis:
    def __init__(self, motor_left, motor_right):
        self.motor_left = motor_left
        self.motor_right = motor_right
        self.bus = smbus.SMBus(1)
        init_motor(self.bus, motor_left)
        init_motor(self.bus, motor_right)

    def move(self, speed_left, speed_right):
        set_speed(self.bus, self.motor_left, speed_left)
        set_speed(self.bus, self.motor_right, speed_right)

    def stop(self):
        self.move(0,0)


def callback(message):
    if message.data == 'up':
        robot.move(100,100)
    elif message.data == 'down':
        robot.move(-100,-100)
    elif message.data == 'left':
        robot.move(-100,100)
    elif message.data == 'right':
        robot.move(100,-100)
    elif message.data == 'stop':
        robot.stop()

def listener():
    rospy.init_node('control_listener')
    rospy.Subscriber('control_data', String, callback)
    rospy.spin()

if __name__ == '__main__':
    #chassis = Chassis(i2c.MOTOR_L, i2c.MOTOR_R)
    robot = Robot(MOTOR_L, MOTOR_R, GYRO)
    listener()
