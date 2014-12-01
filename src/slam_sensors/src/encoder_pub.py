#!/usr/bin/env python
import rospy, sys, i2c
from std_msgs.msg import String
from subprocess import check_output

MOTOR_L = 0x0a
MOTOR_R = 0x0b
GYRO = 0x69

def stream_encoder():
    rospy.init_node('encoder')
    pub = rospy.Publisher('encoder_data', String)

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub_string = i2c.read(GYRO, 0x28)
        pub.publish(pub_string)
        r.sleep()

if __name__ == '__main__':
    try:
        stream_encoder()
    except rospy.ROSInterruptException:
        pass
