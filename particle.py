import numpy as np
from numpy import pi
import random
from particle_testbot import *

# move and predict depend on the actual robot dynamics so these should be changed together with the robot
def move_particles(particles, turn, turn_noise, forward, forward_noise, world_size):
    dist = forward + np.random.normal(0.0, forward_noise, len(particles[0]))
    particles[0] = (particles[0] + turn + np.random.normal(0.0, turn_noise, len(particles[0]))) % (2 * pi)
    particles[1] = (particles[1] + (np.cos(particles[0]) * dist)) % world_size
    particles[2] = (particles[2] + (np.sin(particles[0]) * dist)) % world_size

# move and predict depend on the actual robot dynamics so these should be changed together with the robot
def measurement_prob(particles, measurement, m_noise, landmarks):

    prob = np.ones(len(particles[1]))

    for i in range(len(landmarks)):
        dist = np.sqrt((particles[1] - landmarks[i][0]) ** 2 + (particles[2] - landmarks[i][1]) ** 2)
        prob *= np.exp(- ((dist - measurement[i]) ** 2) / (m_noise ** 2) / 2.0) / np.sqrt(2.0 * pi * m_noise ** 2)

    return prob

class ParticleFilter:
    def __init__(self, grid, noise, n, world_size, landmarks):
        self.noise = noise
        self.landmarks = landmarks
        self.world_size = world_size
        self.N = n
        self.w = np.zeros(self.N)
        self.particles = np.array([np.random.random(n)*g for g in grid])

    def move_particles(self, turn, forward):
        move_particles(self.particles, turn, self.noise[1], forward, self.noise[0], self.world_size)

    def calculate_probabilities(self, sensor_data):
        self.w = measurement_prob(self.particles, sensor_data, self.noise[2], self.landmarks)

    def resample_particles(self):

        mw = np.max(self.w)
        index = int(random.random() * self.N)
        beta = 0.0
        particles_temp = self.particles.copy()

        for i in range(self.N):
            beta += random.random() * 2.0 * mw
            while beta > self.w[index]:
                beta -= self.w[index]
                index = (index + 1) % self.N
            self.particles[:, i] = particles_temp[:, index]


def eval_particle_filter(r, p, world_size):
    dx = (p[1] - r.x + (world_size / 2.0)) % world_size - (world_size / 2.0)
    dy = (p[2] - r.y + (world_size / 2.0)) % world_size - (world_size / 2.0)
    err = np.sqrt(dx * dx + dy * dy)
    return sum(err)/len(err)






