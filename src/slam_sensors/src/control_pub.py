#!/usr/bin/env python
import rospy,sys,tty,termios
from std_msgs.msg import String

def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def get():
        k = getch()
        if k=='w':
                return "up"
        elif k=='s':
                return "down"
        elif k=='d':
                return "right"
        elif k=='a':
                return "left"
        else:
                return "stop"

def stream_controls():
    rospy.init_node('control')
    pub = rospy.Publisher('control_data', String)

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = get()
        pub.publish(msg)
        r.sleep()


if __name__ == '__main__':
    try:
        stream_controls()
    except rospy.ROSInterruptException:
        pass
