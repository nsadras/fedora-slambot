#!usr/bin/env python
import numpy as np 
import ros
import time

import Listener

NUM_LANDMARKS = 16

class Slam:
    def __init__(self, numLandmarks):
        self.landmarks = [] 
        self.numLandmarks = numLandmarks
        self.markerQueue = []
        self.dataQueue = []
        self.position = np.matrix( np.zeros(2 * numLandmarks + 3) ).T
        self.cov = np.matrix( np.zeros( (2 * numLandmarks + 3, 2 * numLandmarks + 3) ) )
        self.lastSample = time.clock()

    def update(self, robotState, observations = False):

    #Prediction Phase

        #Updating Mean
        transV, rotV = robotState[0], robotState[1]
        timeSlice = time.clock() - self.lastSample
        theta = self.position[2,0]
        F = np.matrix(np.c_[np.eye(3) , np.zeros((3,2 * self.numLandmarks))])
        position_update, position_update_J = 0,0    #Just initialization TODO:MAKE THIS MORE ELEGANT WTF
        if not rotV == 0:
            position_update = np.matrix([-transV/float(rotV) * np.sin(theta) + transV/float(rotV) * np.sin(theta + rotV * timeSlice),
                                         transV/float(rotV) * np.cos(theta) - transV/float(rotV) * np.cos(theta + rotV * timeSlice),
                                         rotV * timeSlice]).T
        else:
            position_update = np.matrix([transV * timeSlice * np.sin(theta),
                                         transV * timeSlice * np.cos(theta),
                                         0.]).T
        position_bar = self.position + F.T * position_update

        #Updating Covariance
        if not rotV == 0:
            position_update_J = np.matrix([-transV/float(rotV) * np.cos(theta) + transV/float(rotV) * np.cos(theta + rotV * timeSlice),
                                           -transV/float(rotV) * np.sin(theta) + transV/float(rotV) * np.sin(theta + rotV * timeSlice),
                                           0.]).T
        else:
            position_update_J = np.matrix([transV * timeSlice * np.cos(theta),
                                           -transV * timeSlice * np.sin(theta),
                                           0.]).T
        J = np.matrix( np.c_[np.zeros((3,2)) , position_update_J ] )
        G = np.eye(2 * self.numLandmarks + 3) + F.T * J * F
        cov_bar = G * self.cov * G.T #TODO: + NOISE MATRIX 

    #Correction Phase
       
        #Compute Kalman Gain
        if observations is not False:
            for observation in observations:
                letter, distance = observation[0],observation[1]      #Subject to change based on format of observation
                obs_land_x = position_bar[0,0] + distance * np.cos(position_bar[2,0])   
                obs_land_y = position_bar[1,0] + distance * np.sin(position_bar[2,0])

                #First observation of landmark
                if letter not in self.landmarks and not len(self.landmarks) == self.numLandmarks:
                    position_bar[len(self.landmarks) + 3] = obs_land_x
                    position_bar[len(self.landmarks) + 4] = obs_land_y
                    cov_bar[2*len(self.landmarks)+3,2*len(self.landmarks)+3] = 1.
                    cov_bar[2*len(self.landmarks)+4,2*len(self.landmarks)+4] = 1.
                    self.landmarks.append(letter)
                #Already saw the landmark somewhere
                else:
                    index = self.landmarks.index(letter)
                    diff_x,diff_y = obs_land_x - position_bar[2*index + 3,0], obs_land_y - position_bar[2*index + 4,0]
                    difference = np.matrix([[diff_x] , [diff_y]])
                    euclid_dist = np.sqrt(np.power(diff_x , 2) + np.power(diff_y , 2))

                    #Constructing Shift Matrix Accordingly
                    J_gain = 1./euclid_dist * np.matrix([[-euclid_dist*diff_x , -euclid_dist*diff_y ,0.0 , euclid_dist*diff_x ,euclid_dist*diff_y], [diff_y, -diff_x, -np.power(euclid_dist,2), -diff_y, diff_x]])
                    shiftMat = np.array(np.zeros((5 , 2 * self.numLandmarks + 3)))
                    shiftMat[0][0], shiftMat[1][1], shiftMat[2][2] = 1. , 1. , 1.
                    shiftMat[3][index * 2 + 3] , shiftMat[4][index * 2 + 4] = 1., 1.
                    J_gain = J_gain * shiftMat

                    #Kalman Gain Update
                    #TODO: ADD SENSOR NOISE
                    kalman_gain = cov_bar * J_gain.T * np.linalg.inv(J_gain * cov_bar * J_gain.T + np.eye(2)*1.0e2)
                    position_bar = position_bar + kalman_gain * difference 
                    cov_bar = (np.eye(self.numLandmarks * 2 + 3) - kalman_gain * J_gain) * cov_bar

        #Update
        self.lastSample = time.clock()
        self.position = position_bar
        self.cov = cov_bar

def main():
    slamRobot = new Slam(NUM_LANDMARKS)
    dataListener = Listener('sensorListener','gyro_data',slamRobot.dataQueue) 
    markListener = Listener('camDataListener','markers',slamRobot.markerQueue) 
    while True:
        if len(slamRobot.dataQueue) >= 1 and len(slamRobot.markerQueue) >= 1:
            data = dataQueue.pop()
            markers = markerQueue.pop()
            slamRobot.update(data,observations=markers)

if __name__ == '__main__': main()
