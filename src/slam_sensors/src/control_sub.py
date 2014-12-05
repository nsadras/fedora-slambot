#!/usr/bin/env python

import rospy, i2c, smbus
from std_msgs.msg import String
from robot import *

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
    robot = Robot(MOTOR_L, MOTOR_R, GYRO)
    listener()
