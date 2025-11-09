"""
Extended State Observer (ESO) implementation for ADRC.
"""

class ESO:
    """
    Extended State Observer for estimating state and total disturbance.

    State variables:
        x1: position estimate
        x2: velocity estimate
        d: total disturbance estimate
    """

    def __init__(self, Ts, b_hat, w_o):
        """
        Initialize ESO.

        Args:
            Ts: Sampling time (seconds)
            b_hat: Control gain estimate
            w_o: Observer bandwidth
        """
        self.Ts = Ts
        self.b_hat = b_hat
        self.w_o = w_o

        # Observer gains
        self.beta1 = 3.0 * w_o
        self.beta2 = 3.0 * w_o ** 2
        self.beta3 = w_o ** 3

        # State estimates
        self.x1 = 0.0  # position estimate
        self.x2 = 0.0  # velocity estimate
        self.d = 0.0   # total disturbance estimate

    def update(self, y, u):
        """
        Update the observer with new measurement and control input.

        Args:
            y: measured position
            u: control input
        """
        # Position error
        e = y - self.x1

        # Observer dynamics (Euler integration)
        x1_dot = self.x2 + self.beta1 * e
        x2_dot = self.d + self.b_hat * u + self.beta2 * e
        d_dot = self.beta3 * e

        # Update states
        self.x1 += x1_dot * self.Ts
        self.x2 += x2_dot * self.Ts
        self.d += d_dot * self.Ts
