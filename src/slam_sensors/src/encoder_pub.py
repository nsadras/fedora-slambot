#!/usr/bin/env python
import rospy, sys, i2c
from std_msgs.msg import String
from slam_sensors.msg import RobotVelocity
from subprocess import check_output

MOTOR_L = 0x0a
MOTOR_R = 0x0b
GYRO = 0x68
bus = smbus.SMBus(1)

def to_signed_int(bits):
    if not (bits >> 15) & 1:
        return bits
    else:
        return -(((~bits) & 0xffff) + 1)

def stream_encoder():
    rospy.init_node('encoder')
    pub = rospy.Publisher('encoder_data', String)

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        high = i2c.read(GYRO, 0x47).strip("\n")
	low = i2c.read(GYRO, 0x48)[2:]
	raw = int(high + low, 16)
        msg = RobotVelocity()
	angular = (to_signed_int(raw) / 114)*.0174 # convert from raw data to radians per second	

        left_velocity = i2c.readl(MOTOR_L, 0x04)
        right_velocity = i2c.readl(MOTOR_R, 0x04)

        print "left: %s right: %s" % (left_velocity, right_velocity)

        msg.angular_velocity = angular
        msg.linear_velocity = right_velocity
        pub.publish(str(msg))
        r.sleep()

if __name__ == '__main__':
    #i2c.write(GYRO, 0x20, 0x1f)
    #i2c.write(GYRO, 0x22, 0x08)
    #i2c.write(GYRO, 0x24, 0x80)
    i2c.write(bus, GYRO, 0x6b, 0x00)
    try:
        stream_encoder()
    except rospy.ROSInterruptException:
        pass
