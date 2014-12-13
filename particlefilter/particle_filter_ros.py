from particle_filter_slam import ParticleFilterSLAM
import rospy
from json import loads
from std_msgs.msg import String
from gui import *
from pyglet.window import mouse

window = pyglet.window.Window(WIDTH, HEIGHT)

linear = None
angular = None

def on_sensors(message):
    global linear, angular
    linear, angular, timestamp = loads(message.data)
    print linear, angular
    pf.DynamicsUpdate(linear,angular,timestamp)

def on_markers(message):
    markers = loads(message.data)[0]
    for marker in markers:
        marker_id, pos = marker
        pf.MarkerUpdate(marker_id, pos)

def draw(dt):
    global scale
    window.clear()
    DrawRobot(WIDTH/2, HEIGHT/2, np.pi/2)
    for i in range(len(pf.markers)):
        if pf.markers[i].seen:
            x,y = pf.markers[i].GetPosition()
            pyglet.gl.glColor3f(0.0,1.0,0.0)
            cloud = (pf.markers[i].particles*scale + [WIDTH/2, HEIGHT/2]).flatten()
            pyglet.graphics.draw(len(cloud)/2, pyglet.gl.GL_POINTS,
                ('v2f', cloud)
            )
            DrawMarker(WIDTH/2 + (x)*scale, HEIGHT/2 + (y)*scale, i)

            #for x,y in pf.markers[i].particles:
                #DrawMarker(WIDTH/2 + (x)*scale, HEIGHT/2 + (y)*scale, i)
    if 'q' in keys and keys['q']:
        exit()
    if 'o' in keys and keys['o']:
        scale += 1
    if 'p' in keys and keys['p']:
        if scale > 1:
            scale -= 1
    pyglet.gl.glColor3f(1.0,1.0,1.0)
    #pyglet.graphics.draw(2, pyglet.gl.GL_LINES, ('v2i', (WIDTH/4, HEIGHT/4, WIDTH/4 + scale, HEIGHT/4)))
    if not(linear is None):
        pyglet.gl.glColor3f(1.0,1.0,1.0)
        pyglet.graphics.draw(2, pyglet.gl.GL_LINES, ('v2f', (WIDTH/2, HEIGHT/2, WIDTH/2, HEIGHT/2 + linear*scale)))

keys = {}
@window.event
def on_key_press(symbol, modifiers):
    keys[chr(symbol % 256)] = True
@window.event
def on_key_release(symbol, modifiers):
    keys[chr(symbol % 256)] = False
@window.event
def on_mouse_press(x, y, button, modifiers):
    if button == mouse.LEFT:
        x = (x-WIDTH/2)/float(scale)
        y = (y-HEIGHT/2)/float(scale)
        print x,y,(x*x+y*y)**.5



if __name__ == '__main__':
    pf = ParticleFilterSLAM(16,400)
    rospy.init_node('particle_filter')
    rospy.Subscriber('/sensors', String, on_sensors, queue_size = 1)
    rospy.Subscriber('/markers', String, on_markers, queue_size = 1)
    pyglet.clock.schedule_interval(draw, 0.01)
    pyglet.app.run()
