#!/usr/bin/env python
import rospy, sys, i2c, smbus
from std_msgs.msg import String
from robot import *
from json import dumps

def stream_data():
    pub = rospy.Publisher('gyro_data', String)

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        angular = robot.get_angular_velocity()
        linear = robot.get_linear_velocity()
        if angular is not None and linear is not None:
            pub.publish(dumps((linear,angular)))
        else:
            print "i2c read error"
        r.sleep()

def sub_callback(message):
    if message.data == 'stop':
        robot.is_stopped = True
    else:
        robot.is_stopped = False

if __name__ == '__main__':
    robot = Robot(MOTOR_L, MOTOR_R, GYRO)
    robot.calibrate_sensors(2000)

    rospy.init_node('gyro')
    rospy.Subscriber('control_data', String, sub_callback)
    try:
        stream_data()
    except rospy.ROSInterruptException:
        pass
