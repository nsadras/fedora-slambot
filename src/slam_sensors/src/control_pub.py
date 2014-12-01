#!/usr/bin/env python
import rospy,sys,tty,termios
from std_msgs.msg import String
import pyglet
 
interface = pyglet.window.Window(200, 200)

@interface.event
def on_key_press(symbol, modifiers):
    k = chr(symbol % 256)
    if k=='w':
            pub.publish("up")
    elif k=='s':
            pub.publish("down")
    elif k=='d':
            pub.publish("right")
    elif k=='a':
            pub.publish("left")
    r.sleep()

@interface.event
def on_key_release(symbol, modifiers):
    pub.publish('stop')

if __name__ == '__main__':
    rospy.init_node('control')
    pub = rospy.Publisher('control_data', String, queue_size=1)
    r = rospy.Rate(10)
    pyglet.app.run()
