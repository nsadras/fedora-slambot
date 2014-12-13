import pyglet
import numpy as np
from json import loads
from particle_filter_slam import *
import IPython

WIDTH = 1366
HEIGHT = 768


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

scale = 200


n = 0
def update(dt):
    global scale,n
    window.clear()
    DrawRobot(WIDTH/2, HEIGHT/2, np.pi/2)
    for i in range(len(pf.markers)):
        for marker in pf.markers[i].particles:
            DrawMarker(WIDTH/2 + (marker[0])*scale, HEIGHT/2 + (marker[1])*scale, i)

    t = n/100.0
    if n < 100:
        pf.DynamicsUpdate(0.0,1.0,t)
    elif n < 150:
        pf.DynamicsUpdate(8.0,0.0,t)
    elif n == 150:
        for _ in range(150):
            pf.DynamicsUpdate(0.0,0.0,t)
            pf.MarkerUpdate(0,(1.0,1.0))
            print pf.markers[0].GetPosition()
    elif n < 200:
        pf.DynamicsUpdate(0.0,0.0,t)

    n += 1

    if 'q' in keys and keys['q']:
        exit()
    if 'o' in keys and keys['o']:
        scale += 1
    if 'p' in keys and keys['p']:
        if scale > 1:
            scale -= 1
    pyglet.graphics.draw(2, pyglet.gl.GL_LINES, ('v2i', (WIDTH/4, HEIGHT/4, WIDTH/4 + scale, HEIGHT/4)))


if __name__ == '__main__':
    window = pyglet.window.Window(WIDTH, HEIGHT)
    keys = {}
    @window.event
    def on_key_press(symbol, modifiers):
        keys[chr(symbol % 256)] = True
    @window.event
    def on_key_release(symbol, modifiers):
        keys[chr(symbol % 256)] = False

    pf = ParticleFilterSLAM(1,100)
    pyglet.clock.schedule_interval(update, 0.01)
    pyglet.app.run()
