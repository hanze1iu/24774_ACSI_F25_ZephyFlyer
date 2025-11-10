"""
Extended State Observer (ESO) implementation for ADRC.
"""
import numpy as np

class ESO:
    def __init__(self, Ts, b_hat, w_o):
        self.Ts = Ts
        self.b_hat = b_hat
        self.w_o = w_o

        # === Gains ===
        self.beta1 = 3.0 * w_o
        self.beta2 = 3.0 * (w_o ** 2)
        self.beta3 = (w_o ** 3)

        # === Initial states ===
        self.x1 = 0.0  # position estimate
        self.x2 = 0.0  # velocity estimate
        self.d  = 0.0  # disturbance estimate

    def update(self, y, u):
        """Propagate ESO one step forward with overflow protection."""
        e = y - self.x1

        # Continuous-time ESO model (Euler)
        x1_dot = self.x2 + self.beta1 * e
        x2_dot = self.d + self.b_hat * u + self.beta2 * e
        d_dot  = self.beta3 * e

        # Integrate
        self.x1 += x1_dot * self.Ts
        self.x2 += x2_dot * self.Ts
        self.d  += d_dot  * self.Ts

        # Clamp
        self.x1 = np.clip(self.x1, -2.0, 2.0)
        self.x2 = np.clip(self.x2, -5.0, 5.0)
        self.d  = np.clip(self.d, -10.0, 10.0)

        # Reset if NaN/Inf
        if (not np.isfinite(self.x1)
            or not np.isfinite(self.x2)
            or not np.isfinite(self.d)):
            print(f"[ESO RESET] NaN detected! Resetting observer: y={y:.3f}, u={u:.3f}")
            self.x1 = y
            self.x2 = 0.0
            self.d  = 0.0
