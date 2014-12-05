import pyglet
import numpy as np
import rospy
import json

game_window = pyglet.window.Window(800, 600)
robotImage = pyglet.image.load('robot.jpg')
robot = pyglet.sprite.Sprite(robotImage)

MARKER_RADIUS = 20.0

class RobotState:
    def __init__(self, numLandmarks):
        self.x = 200
        self.y = 400
        self.theta = 0
        self.maxLandmarks = numLandmarks
        self.markers = {}
        self.cov = np.matrix( np.zeros( (2 * numLandmarks + 3 , 2 * numLandmarks + 3) ) )
    
    #Markers are formated as (name , (x,y))
    def drawMarkers(self):
        for marker in self.markers:
            xCoord,yCoord = self.markers[marker]
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
        self.x = new_postion[0]
        self.y = new_postion[1]
        self.theta = new_postion[2]
        self.cov = cov

    def drawPosUncertainty(self):
        return
    
    def drawMarkerUncertainty(self):
        return

robState = RobotState(2)

def callback(message):
    currLandmarks, mu, cov = json.loads(message)    
    flat_mu = np.array(mu).flatten()
    position, posLandmarks = flat_mu[:3].tolist(), flat_mu[3:].tolist()
    landmarksPos = []
    for i in range(len(currLandmarks)):
        landmarksPos.append((posLandmarks[i],posLandmarks[i+1]))
    robState.updateState(position, cov)
    robState.updateMarkers(currLandmarks,landmarksPos)


def update(dt):
    game_window.clear()
    robot.x = robState.x
    robot.y = robState.y
    robot.draw()
    robState.drawMarkers()

def main():
    rospy.init_node('slam_gui')
    rospy.Subscriber('/slam_data', String, callback)
    pyglet.clock.schedule_interval(update, 0.01)
    pyglet.app.run()

if __name__ == '__main__': main()
