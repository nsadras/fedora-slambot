#!usr/bin/env python
import numpy as np 
import rospy
import time
import json
from std_msgs.msg import String

from Listener import Listener

NUM_LANDMARKS = 16
HOMO_VAR = 0.05

GYRO_VAR = 0.05

class Slam:
    def __init__(self, numLandmarks):
        self.landmarks = [] 
        self.numLandmarks = numLandmarks
        self.markerQueue = []
        self.dataQueue = []
        self.position = np.matrix( np.zeros(2 * numLandmarks + 3) ).T
        self.position[2,0] = np.pi/2.
        self.cov = np.matrix( np.zeros( (2 * numLandmarks + 3, 2 * numLandmarks + 3) ) )
        self.pub = rospy.Publisher('/slam_data',String, queue_size =1)
        #self.rate = rospy.Rate(20)
        self.lastSample = 0.0

    def update(self, robotState, observations = False):
        if self.lastSample == 0:
            self.lastSample = robotState[2]

    #Prediction Phase

        #Updating Mean
        transV, rotV, currSample = robotState[0], robotState[1], robotState[2]
        timeSlice = (currSample - self.lastSample)
        self.lastSample = currSample
        theta = self.position[2,0]
        F = np.matrix(np.c_[np.eye(3) , np.zeros((3,2 * self.numLandmarks))])
        position_update, position_update_J = 0,0     
        if not rotV == 0:
            position_update = np.matrix([-transV/float(rotV) * np.sin(theta) + transV/float(rotV) * np.sin(theta + rotV * timeSlice),
                                         transV/float(rotV) * np.cos(theta) - transV/float(rotV) * np.cos(theta + rotV * timeSlice),
                                         rotV * timeSlice]).T
        else:
            position_update = np.matrix([transV * timeSlice * np.sin(theta),
                                         -transV * timeSlice * np.cos(theta),
                                         0.]).T
        noise_matrix = np.matrix([[np.random.normal(0,.2,1)[0],0,0],[0,np.random.normal(0,.2,1)[0],0],[0,0,np.random.normal(0,GYRO_VAR,1)[0]]])
        position_bar = self.position + F.T * position_update

        #Updating Covariance
        if not rotV == 0:
            position_update_J = np.matrix([ transV/float(rotV) * np.cos(theta) - transV/float(rotV) * np.cos(theta + rotV * timeSlice),
                                            transV/float(rotV) * np.sin(theta) - transV/float(rotV) * np.sin(theta + rotV * timeSlice),
                                           0.]).T
        else:
            position_update_J = np.matrix([-transV * timeSlice * np.cos(theta),
                                           -transV * timeSlice * np.sin(theta),
                                           0.]).T
        J = np.matrix( np.c_[np.zeros((3,2)) , position_update_J ] )
        G = np.eye(2 * self.numLandmarks + 3) + F.T * J * F
        cov_bar = G * self.cov * G.T + F.T * noise_matrix * F 

    #Correction Phase
        #Compute Kalman Gain
        if observations is not False:
            for observation in observations:
                letter, newX, newY = observation[0], observation[1][0], observation[1][1]
                distance = np.sqrt(newX ** 2 + newY ** 2)
                letterAngle = np.arctan2(newY,newX) % (2.0*np.pi)
                obs_land_x = position_bar[0,0] + distance * np.cos(position_bar[2,0] + letterAngle)   
                obs_land_y = position_bar[1,0] + distance * np.sin(position_bar[2,0] + letterAngle)
                obs_distance = np.sqrt(obs_land_x ** 2 + obs_land_y ** 2)

                #First observation of landmark
                if letter not in self.landmarks and not len(self.landmarks) == self.numLandmarks:
                    position_bar[2*len(self.landmarks) + 3] = obs_land_x
                    position_bar[2*len(self.landmarks) + 4] = obs_land_y
                    cov_bar[2*len(self.landmarks)+3,2*len(self.landmarks)+3] = 1.
                    cov_bar[2*len(self.landmarks)+4,2*len(self.landmarks)+4] = 1.
                    self.landmarks.append(letter)
                #Already saw the landmark somewhere
                else:
                    index = self.landmarks.index(letter)
                    diff_x,diff_y = obs_land_x - position_bar[2*index + 3,0] , obs_land_y - position_bar[2*index + 4,0]
                    euclid_dist = np.sqrt(diff_x ** 2 + diff_y ** 2)
                    angleDiff = (np.arctan2(diff_y,diff_x) - position_bar[2,0]) % (2.*np.pi)

                    #Constructing Shift Matrix Accordingly
                    J_gain = 1./(euclid_dist**2) * np.matrix([[euclid_dist*diff_x , -euclid_dist*diff_y ,0.0 , -euclid_dist*diff_x ,euclid_dist*diff_y], [diff_y, diff_x, -1., -diff_y, -diff_x]])
                    shiftMat = np.array(np.zeros((5 , 2 * self.numLandmarks + 3)))
                    shiftMat[0][0], shiftMat[1][1], shiftMat[2][2] = 1. , 1. , 1.
                    shiftMat[3][index * 2 + 3] , shiftMat[4][index * 2 + 4] = 1., 1.
                    J_gain = J_gain * np.matrix(shiftMat)

                    #Kalman Gain Update
                    noise = np.random.normal(0.0,GYRO_VAR,1)[0]
                    kalman_gain = np.matrix(cov_bar * J_gain.T * np.linalg.inv(J_gain * cov_bar * J_gain.T + np.matrix(np.eye(2)*noise)))
                    position_bar = position_bar + kalman_gain * (np.matrix([obs_distance - euclid_dist, (letterAngle - angleDiff) % (2.0 * np.pi)]).T) 
                    cov_bar = np.matrix((np.eye(self.numLandmarks * 2 + 3) - kalman_gain * J_gain)) * cov_bar

        #Update
        self.position = position_bar
        self.position[2] = self.position[2] % (2 * np.pi)
        self.cov = cov_bar
        self.publish()

    def publish(self):
        #Preprocessing
        message = json.dumps([self.landmarks,self.position.flatten().tolist(),self.cov.tolist()])
        self.pub.publish(message)

def main():
    print "Initializing RosNode..."
    rospy.init_node('slam')
    slamRobot = Slam(NUM_LANDMARKS)
    print "Subscribing to Robot Data.."
    dataListener = Listener('gyro_data',slamRobot.dataQueue) 
    print "Subscribing to Vision Data.."
    markListener = Listener('/markers',slamRobot.markerQueue) 
    while not rospy.is_shutdown():
        if (len(slamRobot.dataQueue)) >= 1 and (len(slamRobot.markerQueue)) >= 1:
            data = slamRobot.dataQueue.pop(0)
            markers = slamRobot.markerQueue.pop(0)
            slamRobot.update(data,observations=markers)
            print slamRobot.position

if __name__ == '__main__': main()
