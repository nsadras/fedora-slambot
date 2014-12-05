#!/usr/bin/env python
import rospy, sys, i2c, smbus
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

        msg = RobotVelocity()

        gyro_high = i2c.read(GYRO, 0x47).strip("\n")
	gyro_low = i2c.read(GYRO, 0x48)[2:]
	gyro_raw = int(gyro_high + gyro_low, 16)
	angular = (to_signed_int(gyro_raw) / 114)*.0174 # convert from raw data to radians per second	

        accel_high = i2c.read(GYRO, 0x3d).strip("\n")
        accel_low = i2c.read(GYRO, 0x3e)[2:]
        accel_raw = int(accel_high + accel_low, 16)
        linear = (to_signed_int(accel_raw))

        msg.angular_velocity = angular
        msg.linear_velocity = linear
        pub.publish(str(msg))
        r.sleep()

if __name__ == '__main__':
    i2c.write(bus, GYRO, 0x6b, 0x00) # activate sensor
    try:
        stream_encoder()
    except rospy.ROSInterruptException:
        pass
