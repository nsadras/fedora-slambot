#!/usr/bin/env python
import rospy, sys, i2c, smbus
from std_msgs.msg import String
from slam_sensors.msg import RobotVelocity
from robot import *

def stream_data():
    rospy.init_node('gyro')
    pub = rospy.Publisher('gyro_data', RobotVelocity)

    r = rospy.Rate(10)

    while not rospy.is_shutdown():

        msg = RobotVelocity()

        msg.angular_velocity = robot.get_angular_velocity()
        msg.linear_velocity = robot.get_linear_velocity()
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    robot = Robot(MOTOR_L, MOTOR_R, GYRO)
    try:
        stream_data()
    except rospy.ROSInterruptException:
        pass
