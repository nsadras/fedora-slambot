#!/usr/bin/env python
import rospy,sys,tty,termios
from std_msgs.msg import String

import pyglet
 
interface = pyglet.window.Window(800, 600)
keys = {}
 
@interface.event
def on_key_press(symbol, modifiers):
    keys[chr(symbol % 256)] = True
 
@interface.event
def on_key_release(symbol, modifiers):
    keys[chr(symbol % 256)] = False
 
def stream_controls():
    rospy.init_node('control')
    pub = rospy.Publisher('control_data', String)

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        if keys["w"]:
            msg = "up"
        elif keys["s"]:
            msg = "down"
        elif keys["a"]:
            msg = "left"
        elif keys["d"]:
            msg = "right"
        else:
            msg = "stop"
        pub.publish(msg)
        r.sleep()


if __name__ == '__main__':
    pyglet.app.run()
    try:
        stream_controls()
    except rospy.ROSInterruptException:
        pass
