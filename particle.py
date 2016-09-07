import numpy as np
from numpy import pi
import random

class Robot:
    def __init__(self):
        self.x = np.random.random() * world_size
        self.y = np.random.random() * world_size
        self.orientation = np.random.random() * 2.0 * pi
        self.forward_noise = 0.0
        self.turn_noise = 0
        self.sense_noise = 0

    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError('X coordinate out of bound')
        if new_y < 0 or new_y >= world_size:
            raise ValueError('Y coordinate out of bound')
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise)
        self.turn_noise = float(new_t_noise)
        self.sense_noise = float(new_s_noise)

    def sense(self):
        z = []
        for i in range(len(landmarks)):
            dist = np.sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            z.append(dist)
        return z

    def move(self, turn, forward):
        if forward < 0:
            raise ValueError('Robot cant move backwards')

            # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi

        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (np.cos(orientation) * dist)
        y = self.y + (np.sin(orientation) * dist)
        x %= world_size  # cyclic truncate
        y %= world_size

        # set particle
        res = Robot()
        res.set(x, y, orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        return res

    def Gaussian(self, mu, sigma, x):

        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return np.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / np.sqrt(2.0 * pi * (sigma ** 2))

    def measurement_prob(self, measurement):

        # calculates how likely a measurement should be

        prob = 1.0
        for i in range(len(landmarks)):
            dist = np.sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
        return prob

    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))


def eval(r, p):
    s = 0.0
    for i in range(len(p)):  # calculate mean error
        dx = (p[i].x - r.x + (world_size / 2.0)) % world_size - (world_size / 2.0)
        dy = (p[i].y - r.y + (world_size / 2.0)) % world_size - (world_size / 2.0)
        err = np.sqrt(dx * dx + dy * dy)
        s += err
    return s / float(len(p))




class ParticleFilter:
    def __init__(self, f_noise, t_noise, s_noise, n):
        self.N = n
        self.p = []
        self.w = []
        for i in range(N):
            r = Robot()
            r.set_noise(f_noise, t_noise, s_noise)
            self.p.append(r)

    def move_particles(self, turn, forward):
        p2 = []
        for i in range(self.N):
            p2.append(self.p[i].move(0.1, 5.0))
        self.p = p2

    def get_probability(self, Z):
        self.w = []
        for i in range(self.N):
            self.w.append(self.p[i].measurement_prob(Z))

    def resample_particle(self):
        p3 = []
        index = int(random.random() * N)
        beta = 0.0
        mw = max(self.w)
        for i in range(self.N):
            beta += random.random() * 2.0 * mw
            while beta > self.w[index]:
                beta -= self.w[index]
                index = (index + 1) % N
            p3.append(self.p[index])
        self.p = p3




landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0
myrobot = Robot()
myrobot = myrobot.move(0.1, 5.0)
Z = myrobot.sense()
N = 1000
T = 10

f_noise, t_noise, s_noise = 0.05, 0.05, 5.0
turn, forward = 0.1, 5.0


filter = ParticleFilter(f_noise, t_noise, s_noise, N)
for t in range(T):
    myrobot = myrobot.move(turn, forward)
    Z = myrobot.sense()
    filter.move_particles(turn, forward)
    filter.get_probability(Z)
    filter.resample_particle()





    print eval(myrobot, filter.p)


