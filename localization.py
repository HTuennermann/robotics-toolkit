import numpy as np


def move(probability, motion, p_move):
    """ Calculates the result of a movement, this is a circular world"""
    y, x = motion
    return p_move * (np.roll(np.roll(probability, x, axis=1), y, axis=0)) + (1. - p_move) * probability


def detect(probability, measurement, environment, sensor_right):
    """ Calculate probability of a measurement by summing over the probabilities of all the individual measurements"""
    mul = []
    for c in environment:
        mul.append([(sensor_right if i == measurement else 1 - sensor_right) for i in c])

    mul = np.array(mul)

    probability = (mul * probability)
    s = np.sum(probability)
    return probability / s


def localize_on_2d_grid(grid2d, measurements, motions, sensor_right, p_move):

    x = len(grid2d)
    y = len(grid2d[0])

    initial_value = 1.0 / (x*y)

    p = np.zeros([x, y]) + initial_value
    p = np.array(p)

    for measurement, motion in zip(measurements, motions):
        p = move(p, motion, p_move)
        p = detect(p, measurement, grid2d, sensor_right)



    return p






