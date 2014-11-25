import rospy
from std_msgs.msg import String
import sys
from subprocess import check_output

MOTOR_L = 0x0a
MOTOR_R = 0x0b

def i2cread(address, register):
    result = check_output(["i2cget", "-y", "1", str(address), str(register)])
    result.strip("\n")
    return result

def stream_encoder():
    rospy.init_node('encoder')
    pub = rospy.Publisher('encoder_streamer', String)

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub_string = i2cread(MOTOR_L, 0x20)
        pub.publish(pub_string)
        r.sleep()

if __name__ == '__main__':
    try:
        stream_encoder()
    except rospy.ROSInterruptException:
        pass
