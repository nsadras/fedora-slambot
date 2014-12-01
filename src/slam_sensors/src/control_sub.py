#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(message):
    if message.data == 'up':
    elif message.data == 'down':
    elif message.data == 'left':
    elif message.data == 'right':

def listener():
    rospy.init_node('control_listener')
    rospy.Subscriber('control_data', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
