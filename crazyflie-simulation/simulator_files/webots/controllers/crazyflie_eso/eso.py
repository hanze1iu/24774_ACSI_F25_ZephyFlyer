import numpy as np

class ESO:
    def __init__(self, Ts, b_hat, w_o):
        self.Ts = Ts
        self.b_hat = b_hat
        self.l1 = 3 * w_o
        self.l2 = 3 * (w_o ** 2)
        self.l3 = (w_o ** 3)
        self.x1 = 0.0  # state estimate
        self.x2 = 0.0  # derivative estimate
        self.d  = 0.0  # total disturbance

    def update(self, y, u):
        e = y - self.x1
        dx1 = self.x2 + self.l1 * e
        dx2 = self.d + self.b_hat * u + self.l2 * e
        dd  = self.l3 * e
        self.x1 += self.Ts * dx1
        self.x2 += self.Ts * dx2
        self.d  += self.Ts * dd
        return self.x1, self.x2, self.d
