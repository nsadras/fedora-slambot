from scipy import interpolate
import matplotlib.pyplot as plt
import numpy as np

class Integrator:
    def __init__(self):
        self.samples = []
        self.time = []
        self.curr_time = 0.0
        self.integral = 0.0
        self.warmup = False

    def update(self, x, dt):
        self.curr_time += dt
        self.samples.append(x)
        self.time.append(self.curr_time)
        if len(self.samples) > 4:
            self.samples = self.samples[1:]
            self.time = self.time[1:]
            tck = interpolate.splrep(self.time,self.samples)
            if self.warmup:
                self.integral += interpolate.splint(self.time[-2],self.time[-1],tck)
            else:
                self.integral += interpolate.splint(self.time[0],self.time[-1],tck)
                self.warmup = True
    def get(self):
        return self.integral

if __name__ == '__main__'
    t=np.r_[:100]
    x=np.sqrt(t)

    integrator = Integrator()
    for i in range(len(t)):
        if i == 0:
            integrator.update(x[i],t[i])
        else:
            integrator.update(x[i],t[i] - t[i-1])
        print integrator.get()

    tck = interpolate.splrep(t,x)
    print interpolate.splint(t[0],t[-1],tck)
