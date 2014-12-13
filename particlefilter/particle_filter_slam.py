import numpy as np
from threading import Lock
from time import time


RESAMPLE_PROB_THRESHOLD = 0.3
RANDOM_SAMPLE_GRID_DIM = 2.0
MARKER_SIGMA = 0.00001
VELOCITY_SIGMA = 0.0001
ROTATION_SIGMA = 0.00001
RESAMPLE_SIGMA = 0.0125

def Rotate(x,y,theta):
    return x*np.cos(theta) - y*np.sin(theta), x*np.sin(theta) + y*np.cos(theta)

def STDNormal(x):
    return np.exp(-(x/2.0)**2)/(np.sqrt(2*np.pi))

class ParticleFilterMarker:
    def __init__(self, num_particles, random_sample_grid_dim = RANDOM_SAMPLE_GRID_DIM):
        self.random_sample_grid_dim = random_sample_grid_dim
        self.num_particles = num_particles
        self.resample_particles(self.num_particles, self.random_sample_grid_dim)
        self.seen = False
        self.best_position = None

    def resample_particles(self, num_particles, random_sample_grid_dim = RANDOM_SAMPLE_GRID_DIM):
        self.particles = []
        for _ in range(num_particles):
            self.particles.append([2*random_sample_grid_dim*np.random.random() - random_sample_grid_dim,
                                   2*random_sample_grid_dim*np.random.random() - random_sample_grid_dim])
        self.particles = np.array(self.particles)
    def DynamicsUpdate(self, vtheta, v, dt, rotation_sigma = ROTATION_SIGMA, velocity_sigma = VELOCITY_SIGMA):
        for particle in self.particles:
            x,y = particle
            x,y = Rotate(x,y,-(vtheta + np.random.randn()*rotation_sigma)*dt)
            y -= (v + np.random.randn()*velocity_sigma)*dt
            x +=  (np.random.randn()*velocity_sigma)*dt
            particle[0] = x
            particle[1] = y

    def MarkerUpdate(self, marker_pos, sigma = MARKER_SIGMA, resample_prob_threshold = RESAMPLE_PROB_THRESHOLD):
        mean_x, mean_y = marker_pos
        weights = np.zeros(len(self.particles))
        total = 0.0
        for i in range(len(self.particles)):
            x,y = self.particles[i]
            dist = ((x-mean_x)**2 + (y-mean_y)**2)**.5
            weight = STDNormal(dist)
            weights[i] = weight
            total += weight
        if max(weights) < resample_prob_threshold:
            self.resample_particles(self.num_particles, self.random_sample_grid_dim)
            self.MarkerUpdate(marker_pos, sigma, resample_prob_threshold)
            return
        weights /= total
        new_particles = []
        for _ in range(len(self.particles)):
            new_particles.append(np.random.choice(np.r_[:len(self.particles)],p=weights))
        self.particles = np.array(self.particles)[new_particles]
        best_weight = 0
        for i in range(len(self.particles)):
            x,y = self.particles[i]
            dist = ((x-mean_x)**2 + (y-mean_y)**2)**.5
            weight = STDNormal(dist)
            if weight > best_weight:
                best_weight = weight
                self.best_position = i
            self.particles[i][0] +=  np.random.randn()*RESAMPLE_SIGMA
            self.particles[i][1] +=  np.random.randn()*RESAMPLE_SIGMA

    def GetPosition(self):
        #if self.best_position is None:
        return np.mean(self.particles, axis=0)
        #else:
            #return self.particles[self.best_position]
    

class ParticleFilterSLAM:
    def __init__(self, num_markers, num_particles):
        self.markers = []
        for _ in range(num_markers):
            self.markers.append(ParticleFilterMarker(num_particles))
        self.last_update = None
        self.theta = 0
        self.lock = Lock()
    def DynamicsUpdate(self, linear_velocity, angular_velocity, timestamp):
        with self.lock:
            if self.last_update is None:
                self.last_update = timestamp
                return
            self.theta += angular_velocity * (timestamp - self.last_update)
            for marker in self.markers:
                marker.DynamicsUpdate(angular_velocity, linear_velocity, timestamp - self.last_update)
            self.last_update = timestamp
    def MarkerUpdate(self, marker_name, marker_position_from_robot):
        with self.lock:
            self.markers[marker_name].MarkerUpdate(marker_position_from_robot)
            self.markers[marker_name].seen = True
