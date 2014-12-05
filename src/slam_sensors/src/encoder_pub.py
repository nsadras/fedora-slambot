#!/usr/bin/env python
import rospy, sys, i2c, smbus
from std_msgs.msg import String
from slam_sensors.msg import RobotVelocity
from robot import *

def stream_data():
    pub = rospy.Publisher('gyro_data', RobotVelocity)

    r = rospy.Rate(10)

    while not rospy.is_shutdown():

        msg = RobotVelocity()

        angular = robot.get_angular_velocity()
        linear = robot.get_linear_velocity()
        if angular is not None and linear is not None:
            msg.angular_velocity = angular
            msg.linear_velocity = linear
            pub.publish(msg)
        else:
            print "i2c read error"
        r.sleep()

def sub_callback(message):
    if message.data == 'stop':
        robot.velocity = 0

if __name__ == '__main__':
    robot = Robot(MOTOR_L, MOTOR_R, GYRO)

    rospy.init_node('gyro')
    rospy.Subscriber('control_data', String, sub_callback)
    try:
        stream_data()
    except rospy.ROSInterruptException:
        pass
