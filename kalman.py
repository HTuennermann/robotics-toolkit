import numpy as np

def update(mean1, var1, mean2, var2):
    new_mean = (var2 * mean1 + var1 * mean2) / (var1 + var2)
    new_var = 1/(1/var1 + 1/var2)
    return [new_mean, new_var]


def predict(mean1, var1, mean2, var2):
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return [new_mean, new_var]



class kalman_filter:
    def __init__(self, F, H, R, u):

        self.u = np.matrix(u)  # external motion
        self.F = np.matrix(F)  # next state function
        self.H = np.matrix(H)  # measurement function
        self.R = np.matrix(R)  # measurement uncertainty
        self.I = np.matrix(np.identity(len(self.F)))

    def update(self, x, P, measurement):

        Z = np.matrix([measurement])
        y = Z.getT() - (self.H * x)
        S = (self.H * P * self.H.getT()) + self.R
        K = (P * self.H.getT() * S.getI())
        x = x + (K * y)
        P = (self.I - (K * self.H)) * P

        return x, P

    def predict(self, x, P):
        x = (self.F * x) + self.u
        P = self.F * P * self.F.getT()
        return x, P

    def kalman_filter(self, x, P, measurements):
        for measurement in measurements:
            x, P = self.update(x, P, measurement)
            x, P = self.predict(x, P)
        return x, P

    def kalman_filter2(self, x, P, measurements):
        for measurement in measurements:
            x, P = self.predict(x, P)
            x, P = self.update(x, P, measurement)
        return x, P







