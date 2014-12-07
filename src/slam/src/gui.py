import pyglet
import numpy as np
import rospy
import json
from std_msgs.msg import String

game_window = pyglet.window.Window(800, 600)
robotImage = pyglet.image.load('robot.jpg')
robot = pyglet.sprite.Sprite(robotImage)
robot.x = 400
robot.y = 300

MARKER_RADIUS = 20.0
TURN_SCALING = 20.
SCALING = 150.

class RobotState:
    def __init__(self, numLandmarks):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0
        self.maxLandmarks = numLandmarks
        self.markers = {}
        self.cov = np.matrix( np.zeros( (2 * numLandmarks + 3 , 2 * numLandmarks + 3) ) )
    
    #Markers are formated as (name , (x,y))
    def drawMarkers(self):
        markersToDraw = self.markers.copy()
        for marker in markersToDraw:
            xCoord,yCoord = self.markers[marker]
            xCoord = SCALING*(self.x - xCoord) + 400
            yCoord = SCALING*(self.y -yCoord) + 300
            label = pyglet.text.Label(str(marker),
                    font_name='Times New Roman',
                    font_size=12,
                    x=xCoord, y=yCoord,
                    anchor_x='center', anchor_y='center')
            polygon = [xCoord + MARKER_RADIUS, yCoord, xCoord, yCoord + MARKER_RADIUS, xCoord - MARKER_RADIUS, yCoord, xCoord, yCoord - MARKER_RADIUS]
            pyglet.gl.glColor3f(0.,0.5,0.)
            pyglet.graphics.draw(4, pyglet.gl.GL_POLYGON, ('v2f', polygon))
            label.draw()

    def updateMarkers(self, markerList, markerState):
        for i in range(len(markerList)):
            if markerList[i] not in markerState and len(self.markers) < self.maxLandmarks:
                self.markers[markerList[i]] = markerState[i]
            else:
                self.markers[markerList[i]] = markerState[i]

    def updateState(self, new_position, cov):
        self.x = new_position[0]
        self.y = new_position[1]
        self.theta = new_position[2]
        self.cov = cov

    def drawPosUncertainty(self):
        return
    
    def drawMarkerUncertainty(self):
        return

robState = RobotState(2)

def callback(message):
    currLandmarks, mu, cov = json.loads(message.data)    
    flat_mu = np.array(mu).flatten()
    position, posLandmarks = flat_mu[:3].tolist(), flat_mu[3:].tolist()
    landmarksPos = []
    for i in range(len(currLandmarks)):
        landmarksPos.append((posLandmarks[i],posLandmarks[i+1]))
    robState.updateState(position, cov)
    robState.updateMarkers(currLandmarks,landmarksPos)

def update(dt):
    game_window.clear()
    robot.rotation = robState.theta * 180./np.pi * TURN_SCALING
    robot.draw()
    robState.drawMarkers()

def main():
    rospy.init_node('slam_gui')
    print "Subscribing to SlamNode..."
    rospy.Subscriber('/slam_data', String, callback)
    pyglet.clock.schedule_interval(update, 0.01)
    pyglet.app.run()

if __name__ == '__main__': main()
