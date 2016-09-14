import unittest
from localization import *
from kalman import *
from particle import *
import numpy as np


class RoboticsTests(unittest.TestCase):

    def test_2d_loc(self):
        colors = [['R', 'G', 'G', 'R', 'R'],
                  ['R', 'R', 'G', 'R', 'R'],
                  ['R', 'R', 'G', 'G', 'R'],
                  ['R', 'R', 'R', 'R', 'R']]

        measurements = ['G', 'G', 'G', 'G', 'G']
        motions = [[0, 0], [0, 1], [1, 0], [1, 0], [0, 1]]

        result = [[0.01105981, 0.02464042, 0.06799663, 0.04472487, 0.02465153],
                  [0.0071532, 0.01017133, 0.08696596, 0.0798843, 0.00935067],
                  [0.00739737, 0.00894373, 0.11272965, 0.35350723, 0.04065549],
                  [0.00910651, 0.0071532, 0.01434922, 0.04313329, 0.0364256]]

        p = localize_on_2d_grid(colors, measurements, motions, sensor_right=0.7, p_move=0.8)

        self.assertTrue(np.all(np.isclose(p, result)))

    def test_kalman(self):
        measurements = [5., 6., 7., 9., 10.]
        motion = [1., 1., 2., 1., 1.]
        measurement_sig = 4.
        motion_sig = 2.
        mu = 0.
        sig = 10000.

        result = np.array([10.999906177177365, 4.005861580844194])


        for measure, mo in zip(measurements, motion):
            mu, sig = update(mu, sig, measure, measurement_sig)
            mu, sig = predict(mu, sig, mo, motion_sig)

        p = np.array([mu, sig])

        self.assertTrue(np.all(np.isclose(p, result)))

    def test_2d_kalman(self):
        """See Udacity Robotics 2.x Example"""
        measurements = [1, 2, 3]

        x = np.matrix([[0.], [0.]])  # initial state (location and velocity)
        P = np.matrix([[1000., 0.], [0., 1000.]])  # initial uncertainty

        x_result = np.matrix([[3.9996664447958645], [0.9999998335552873]])
        P_result = np.matrix([[2.3318904241194827, 0.9991676099921091], [0.9991676099921067, 0.49950058263974184]])

        u = [[0.], [0.]]  # external motion
        F = [[1., 1.], [0, 1.]]  # next state function
        H = [[1., 0.]]  # measurement function
        R = [[1.]]  # measurement uncertainty

        k = kalman_filter(F, H, R, u)




        x, P = k.kalman_filter(x, P, measurements)

        self.assertTrue(np.all(np.isclose(x, x_result)))
        self.assertTrue(np.all(np.isclose(P, P_result)))

    def test_4d_kalman(self):
        x_result = np.matrix([[9.999340731787717],
                              [0.001318536424568617],
                              [9.998901219646193],
                              [-19.997802439292386]])
        P_result = np.matrix([[0.03955609273706198, 0.0, 0.06592682122843721, 0.0],
                    [0.0, 0.03955609273706198, 0.0, 0.06592682122843721],
                    [0.06592682122843718, 0.0, 0.10987803538073201, 0.0],
                    [0.0, 0.06592682122843718, 0.0, 0.10987803538073201]])

        x = np.matrix([[4.], [12.], [0.], [0.]])  # initial state (location and velocity)
        measurements = [[5., 10.], [6., 8.], [7., 6.], [8., 4.], [9., 2.], [10., 0.]]

        P = np.matrix([[0., 0., 0., 0.],
                       [0., 0., 0., 0.],
                       [0., 0., 1000, 0.],
                       [0., 0., 0., 1000]])  # initial uncertainty

        dt = 0.1
        u = [[0.], [0.], [0.], [0.]]  # external motion
        F = [[1., 0., dt, 0.],
             [0., 1., 0., dt],
             [0., 0., 1., 0.],
             [0., 0., 0., 1.]]  # next state function
        H = [[1., 0., 0., 0.], [0., 1., 0., 0.]]  # measurement function
        R = [[0.1, 0.], [0., 0.1]]  # measurement uncertainty

        k = kalman_filter(F, H, R, u)
        x, P = k.kalman_filter2(x, P, measurements)

        self.assertTrue(np.all(np.isclose(x, x_result)))
        self.assertTrue(np.all(np.isclose(P, P_result)))

    def test_particle_filter(self):
        landmarks = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
        world_size = 100.0
        myrobot = Robot(world_size, landmarks)

        f_noise, t_noise, s_noise = 0.05, 0.05, 5
        turn, forward = 0.1, 5.0

        N = 1000
        particle_filter = ParticleFilter([2 * np.pi, world_size, world_size],
                                         [f_noise, t_noise, s_noise], N, world_size, landmarks)

        T = 100
        for t in range(T):
            myrobot = myrobot.move(turn, forward)
            sensor_data = myrobot.sense()

            particle_filter.move_particles(turn, forward)
            particle_filter.calculate_probabilities(sensor_data)
            particle_filter.resample_particles()
            #print eval_particle_filter(myrobot, particle_filter.particles, world_size)

        assert(abs(myrobot.orientation - particle_filter.particles[0].mean()) < 0.4)
        assert(abs(myrobot.x - particle_filter.particles[1].mean()) < 0.4)
        assert(abs(myrobot.y - particle_filter.particles[2].mean()) < 0.4)


if __name__ == '__main__':
    unittest.main()
