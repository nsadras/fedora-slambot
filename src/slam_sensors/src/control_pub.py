#!/usr/bin/env python
import rospy,sys,tty,termios
from std_msgs.msg import String

class _Getch:
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

def get():
        inkey = _Getch()
        while(1):
                k=inkey()
                if k!='':break
        if k=='\x1b[A':
                return "up"
        elif k=='\x1b[B':
                return "down"
        elif k=='\x1b[C':
                return "right"
        elif k=='\x1b[D':
                return "left"
        else:
                return ""

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
