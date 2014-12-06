import pyglet
import rospy
import numpy as np
from std_msgs.msg import String
from json import loads

WIDTH = 800
HEIGHT = 600

window = pyglet.window.Window(WIDTH, HEIGHT)

def Rotate(x,y,theta):
    return x*np.cos(theta) - y*np.sin(theta), x*np.sin(theta) + y*np.cos(theta)

def GetTriangle(x,y,theta):
    LENGTH = 10
    WIDTH = 8
    x0,y0 = Rotate(3*LENGTH/4, 0, theta)
    x1,y1 = Rotate(-LENGTH/4, WIDTH/2, theta)
    x2,y2 = Rotate(-LENGTH/4, -WIDTH/2, theta)
    x0 += x
    x1 += x
    x2 += x
    y0 += y
    y1 += y
    y2 += y
    return [x0,y0,x1,y1,x2,y2]

marker_labels = [pyglet.text.Label(str(i),
                              font_name='Times New Roman',
                              font_size=12,
                              x=0, y=0,
                              anchor_x='center', anchor_y='center') for i in range(16)]


def DrawRobot(x,y,theta):
    pyglet.gl.glColor3f(0.75,1.0,0.75)
    pyglet.graphics.draw(3, pyglet.gl.GL_POLYGON, ('v2f',GetTriangle(x,y,theta)))

def DrawMarker(x,y,identity):
    RADIUS = 8.0
    pyglet.gl.glColor3f(0.5,0,0)
    pyglet.graphics.draw(4, pyglet.gl.GL_POLYGON, ('v2f',[x + RADIUS, y, x, y + RADIUS, x - RADIUS, y, x, y - RADIUS]))
    marker_labels[identity].x = x
    marker_labels[identity].y = y
    marker_labels[identity].draw()

robot_pos = None
marker_coords= None
scale = 200

def callback(message):
    global robot_pos, marker_coords
    window.clear()
    data = loads(message.data)
    labels = data[0]
    x,y,theta = data[1][0][:3]
    label_coords = data[1][0][3:]
    marker_coords = []
    for i in range(len(labels)):
        label_x = (label_coords[2*i] - x)
        label_y = (label_coords[2*i+1] - y)
        marker_coords.append((label_x, label_y, labels[i]))
    robot_pos = (x,y,theta)

def update(dt):
    global scale
    window.clear()
    if robot_pos:
        x,y,theta = robot_pos
        DrawRobot(WIDTH/2, HEIGHT/2, theta)
    if marker_coords:
        for x,y,i in marker_coords:
            DrawMarker(WIDTH/2+x*scale,HEIGHT/2+y*scale,i)
    if 'q' in keys and keys['q']:
        exit()
    if 'o' in keys and keys['o']:
        scale += 1
    if 'p' in keys and keys['p']:
        if scale > 1:
            scale -= 1

keys = {}
@window.event
def on_key_press(symbol, modifiers):
    keys[chr(symbol % 256)] = True
@window.event
def on_key_release(symbol, modifiers):
    keys[chr(symbol % 256)] = False

if __name__ == '__main__':
    rospy.init_node('simple_slam_gui')
    rospy.Subscriber('/slam_data', String, callback)
    pyglet.clock.schedule_interval(update, 0.01)
    pyglet.app.run()
