import pyglet
import numpy as np

from slam import *

game_window = pyglet.window.Window(800, 600)

def Rotate(x,y,theta):
    return x*np.cos(theta) - y*np.sin(theta), x*np.sin(theta) + y*np.cos(theta)

class Robot:
    LENGTH = 10
    WIDTH = 8
    SENSOR_RADIUS = 50
    VELOCITY = 2
    ROTATION_VELOCITY = 0.1
    MARKER_RADIUS=8.0
    def __init__(self):
        self.x = 400
        self.y = 300
        self.theta = 0
        self.markers = []
    def GetTriangle(self):
        x0,y0 = Rotate(3*Robot.LENGTH/4, 0, self.theta)
        x1,y1 = Rotate(-Robot.LENGTH/4, Robot.WIDTH/2, self.theta)
        x2,y2 = Rotate(-Robot.LENGTH/4, -Robot.WIDTH/2, self.theta)
        x0 += self.x
        x1 += self.x
        x2 += self.x
        y0 += self.y
        y1 += self.y
        y2 += self.y
        return [x0,y0,x1,y1,x2,y2]
    def GetCircle(self, iterations = 200):
        points = []
        x = Robot.SENSOR_RADIUS
        y = 0
        step = 2*np.pi/ iterations
        theta = 0
        for _ in range(iterations+1):
            x,y = Rotate(x,y,theta)
            points.append(self.x + x)
            points.append(self.y + y)
            theta += step
        return points
    def draw(self):
        pyglet.gl.glColor3f(0.75,1.0,0.75)
        pyglet.graphics.draw(3, pyglet.gl.GL_POLYGON, ('v2f',self.GetTriangle()))
        pyglet.gl.glColor3f(0.0,1.0,0.0)
        circle = self.GetCircle()
        pyglet.graphics.draw(len(circle)/2, pyglet.gl.GL_POINTS, ('v2f',circle))
        for landmark in Landmark.landmarks:
            dx = self.x - landmark.x
            dy = self.y - landmark.y
            if dx*dx + dy*dy < Robot.SENSOR_RADIUS*Robot.SENSOR_RADIUS:
                pyglet.graphics.draw(2, pyglet.gl.GL_LINES, ('v2f',[self.x,self.y,landmark.x,landmark.y]))
        for marker in self.markers:
            self.DrawMarker(marker[0], marker[1], marker[2], 0.5, 0.5, 1.0)

    def DrawMarker(self, x, y, label, r, g, b):
        geometry = [self.x + x + Robot.MARKER_RADIUS,
                    self.y + y,
                    self.x + x,
                    self.y + y + Robot.MARKER_RADIUS,
                    self.x + x - Robot.MARKER_RADIUS,
                    self.y + y,
                    self.x + x,
                    self.y + y - Robot.MARKER_RADIUS]
        pyglet.gl.glColor3f(r,g,b)
        pyglet.graphics.draw(4, pyglet.gl.GL_POLYGON, ('v2f',geometry))
        if label != None:
            label.x = self.x + x
            label.y = self.y + y
            label.draw()

    def sensors(self, pos_variance = 10.0, landmark_variance = 10.0, vel_variance = 10.0):
        sensed_landmarks = []
        sensed_position = {'x':self.x + np.random.normal(scale=pos_variance), 'y':self.y + np.random.normal(scale=pos_variance)}
        for landmark in Landmark.landmarks:
            dx = landmark.x - self.x
            dy = landmark.y - self.y
            r = np.sqrt(dx ** 2 + dy ** 2)
            if dx*dx + dy*dy < Robot.SENSOR_RADIUS*Robot.SENSOR_RADIUS:
                sensed_landmarks.append([landmark.index,r])
        if 'w' in keys and keys['w']:
            r_mul = 1.0
        elif 's' in keys and keys['s']:
            r_mul = -1.0
        else:
            r_mul = 0.0

        if 'a' in keys and keys['a']:
            theta_mul = 1.0
        elif 'd' in keys and keys['d']:
            theta_mul = -1.0
        else:
            theta_mul = 0.0

        sensed_velocity = [Robot.VELOCITY*r_mul + np.random.normal(scale=vel_variance), Robot.ROTATION_VELOCITY*theta_mul + np.random.normal(scale=vel_variance), 0.03]
        return {'robot':sensed_velocity,'landmarks':sensed_landmarks}
    def up(self):
        dx,dy = Rotate(Robot.VELOCITY,0,self.theta)
        self.x += dx
        self.y += dy
    def down(self):
        dx,dy = Rotate(Robot.VELOCITY,0,self.theta)
        self.x -= dx
        self.y -= dy
    def left(self):
        self.theta += Robot.ROTATION_VELOCITY
    def right(self):
        self.theta -= Robot.ROTATION_VELOCITY



class Landmark:
    RADIUS = 8.0
    num_landmarks = 0
    landmarks = []
    def __init__(self, x, y):
        self.x=x
        self.y=y
        self.index = Landmark.num_landmarks
        Landmark.num_landmarks += 1
        self.label = pyglet.text.Label(str(self.index),
                                  font_name='Times New Roman',
                                  font_size=12,
                                  x=self.x, y=self.y,
                                  anchor_x='center', anchor_y='center')
        Landmark.landmarks.append(self)
    def GetPoly(self):
        return [self.x + Landmark.RADIUS, self.y, self.x, self.y + Landmark.RADIUS, self.x - Landmark.RADIUS, self.y, self.x, self.y - Landmark.RADIUS]
    def draw(self):
        pyglet.gl.glColor3f(0.5,0,0)
        pyglet.graphics.draw(4, pyglet.gl.GL_POLYGON, ('v2f',self.GetPoly()))
        self.label.draw()

robot = Robot()
NUMLANDMARKS = 2
slamRobot = Slam(NUMLANDMARKS)
slamRobot.position[0,0],slamRobot.position[1,0] = robot.x, robot.y

def update(dt):
    game_window.clear()
    robot.draw()
    data = robot.sensors()
    sensors,landmarks = data['robot'],data['landmarks']
    slamRobot.update(sensors,observations = landmarks)
    for landmark in Landmark.landmarks:
        landmark.draw()
    if 'w' in keys and keys['w']:
        robot.up()
    if 'a' in keys and keys['a']:
        robot.left()
    if 's' in keys and keys['s']:
        robot.down()
    if 'd' in keys and keys['d']:
        robot.right()

keys = {}
@game_window.event
def on_key_press(symbol, modifiers):
    keys[chr(symbol % 256)] = True
@game_window.event
def on_key_release(symbol, modifiers):
    keys[chr(symbol % 256)] = False

if __name__ == '__main__':
    Landmark(400,175)
    Landmark(450,175)

    pyglet.clock.schedule_interval(update, 0.01)
    pyglet.app.run()
