import numpy as np
from numpy import pi
import random
from particle_testbot import *


def move_particles(particles, turn, turn_noise, forward, forward_noise):
    dist = forward + np.random.normal(0.0, forward_noise, len(particles[0]))
    particles[0] = (particles[0] + turn + np.random.normal(0.0, turn_noise, len(particles[0]))) % (2 * pi)
    particles[1] = (particles[1] + (np.cos(particles[0]) * dist)) % world_size
    particles[2] = (particles[2] + (np.sin(particles[0]) * dist)) % world_size


def measurement_prob(particles, measurement, m_noise):

    prob = np.ones(len(particles[1]))

    for i in range(len(landmarks)):
        dist = np.sqrt((particles[1] - landmarks[i][0]) ** 2 + (particles[2] - landmarks[i][1]) ** 2)
        prob *= np.exp(- ((dist - measurement[i]) ** 2) / (m_noise ** 2) / 2.0) / np.sqrt(2.0 * pi * m_noise ** 2)

    return prob

class ParticleFilter:
    def __init__(self, grid, noise, n):
        self.noise = noise
        self.N = n
        self.w = np.zeros(self.N)
        self.particles = np.array([np.random.random(N)*g for g in grid])

    def move_particles(self, turn, forward):
        move_particles(self.particles, turn, self.noise[1], forward, self.noise[0])

    def calculate_probabilites(self, sensor_data):
        self.w = measurement_prob(self.particles, sensor_data, self.noise[2])

    def resample_particles(self):

        mw = np.max(self.w)
        index = int(random.random() * N)
        beta = 0.0
        particles_temp = self.particles.copy()

        for i in range(self.N):
            beta += random.random() * 2.0 * mw
            while beta > self.w[index]:
                beta -= self.w[index]
                index = (index + 1) % N
            self.particles[:, i] = particles_temp[:, index]


def eval(r, p):
    dx = (p[1] - r.x + (world_size / 2.0)) % world_size - (world_size / 2.0)
    dy = (p[2] - r.y + (world_size / 2.0)) % world_size - (world_size / 2.0)
    err = np.sqrt(dx * dx + dy * dy)
    return sum(err)/len(err)

landmarks = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0
myrobot = Robot(world_size, landmarks)

N = 1000
T = 100

f_noise, t_noise, s_noise = 0.05, 0.05, 5
turn, forward = 0.1, 5.0

filter = ParticleFilter([2*np.pi, world_size, world_size], [f_noise, t_noise, s_noise], N)

for t in range(T):

    myrobot = myrobot.move(turn, forward)
    sensor_data = myrobot.sense()

    filter.move_particles(turn, forward)
    filter.calculate_probabilites(sensor_data)
    filter.resample_particles()
    print eval(myrobot, filter.particles)




