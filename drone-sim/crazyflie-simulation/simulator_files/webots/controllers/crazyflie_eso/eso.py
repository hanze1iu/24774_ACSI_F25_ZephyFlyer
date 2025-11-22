"""
Full-state Extended State Observer (ESO) for a quadrotor (Crazyflie-style).

State z (18x1):
    z = [ p(3); v(3); euler(3); omega(3); d_f(3); d_tau(3) ]
        p      = [x, y, z] in world frame
        v      = [vx, vy, vz] in world frame
        euler  = [roll, pitch, yaw]
        omega  = [p, q, r]    body angular rates
        d_f    = disturbance force in world frame  [Nx, Ny, Nz]
        d_tau  = disturbance torque in body frame [τx, τy, τz]

Input u (4x1):
    u = [T, tau_x, tau_y, tau_z]^T
        T      = total thrust (body z-axis)
        tau_*  = body torques

Output measurement y_meas (6x1):
    y = [x, y, z, roll, pitch, yaw]^T
"""

import numpy as np


class FullStateESO:
    def __init__(self, Ts, m=29, J, L, g: float = 9.81):
        """
        Parameters
        ----------
        Ts : float
            Sample time [s]
        m : float
            Mass of the vehicle [kg]  
            # MODIFY (use real CF mass)
        J : (3,3) array_like
            Inertia matrix in body frame  
            # MODIFY (use real CF inertia)
        L : (18,6) array_like
            Constant observer gain matrix, designed offline  
            # MODIFY (fill in your designed L)
        g : float, optional
            Gravity [m/s^2]
        """
        self.Ts = float(Ts)
        self.m = float(m)
        self.J = np.array(J, dtype=float).reshape(3, 3)
        self.J_inv = np.linalg.inv(self.J)
        self.g = float(g)
        self.L = np.array(L, dtype=float).reshape(18, 6)

        # z = [p(3); v(3); euler(3); omega(3); d_f(3); d_tau(3)] shape (18,)
        self.z = np.zeros(18, dtype=float)

    # ---------- helper: rotation R(eta) ----------
    def _R(self, eta):
        """Rotation matrix R from body frame to world frame, given Euler angles."""
        phi, theta, psi = eta
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth, sth = np.cos(theta), np.sin(theta)
        cpsi, spsi = np.cos(psi), np.sin(psi)

        R = np.array([
            [cth * cpsi,                 cth * spsi,                -sth],
            [sphi * sth * cpsi - cphi * spsi,
             sphi * sth * spsi + cphi * cpsi,  sphi * cth],
            [cphi * sth * cpsi + sphi * spsi,
             cphi * sth * spsi - sphi * cpsi,  cphi * cth]
        ])
        return R

    # ---------- helper: E(eta) for Euler kinematics ----------
    def _E(self, eta):
        """
        E(eta) such that:
            euler_dot = E(eta) * omega
        where eta = [roll, pitch, yaw], omega = [p, q, r].
        """
        phi, theta, _ = eta
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth, sth = np.cos(theta), np.sin(theta)

        E = np.array([
            [1.0, sphi * np.tan(theta),      cphi * np.tan(theta)],
            [0.0, cphi,                     -sphi],
            [0.0, sphi / cth,                cphi / cth]
        ])
        return E

    # ---------- continuous-time dynamics f(z, u) ----------
    def f_continuous(self, z, u):
        z = np.asarray(z, dtype=float).reshape(18)
        u = np.asarray(u, dtype=float).reshape(4)

        p = z[0:3]
        v = z[3:6]
        eta = z[6:9]
        omega = z[9:12]
        d_f = z[12:15]
        d_tau = z[15:18]

        T = u[0]
        tau = u[1:4]

        R = self._R(eta)
        E = self._E(eta)

        dp = v

        dv = (R @ np.array([0.0, 0.0, T])) / self.m \
             - np.array([0.0, 0.0, self.g]) \
             + d_f

        deta = E @ omega

        domega = self.J_inv @ (tau - np.cross(omega, self.J @ omega) + d_tau)

        dd_f = np.zeros(3)
        dd_tau = np.zeros(3)

        return np.concatenate([dp, dv, deta, domega, dd_f, dd_tau])

    # ---------- numerical Jacobians ----------
    def jacobian_x(self, z, u, eps: float = 1e-5):
        """
        A = df/dz (18x18) via finite differences, around (z, u).
        # MODIFY: Use only for analysis / tuning, not in control loop
        """
        z = np.asarray(z, dtype=float).reshape(18)
        u = np.asarray(u, dtype=float).reshape(4)

        n = z.size
        A = np.zeros((n, n))
        f0 = self.f_continuous(z, u)

        for i in range(n):
            zp = z.copy()
            zp[i] += eps
            fi = self.f_continuous(zp, u)
            A[:, i] = (fi - f0) / eps

        return A

    def jacobian_u(self, z, u, eps: float = 1e-5):
        """
        B = df/du (18x4) via finite differences, around (z, u).
        # MODIFY: Same as A — not needed in real-time loop
        """
        z = np.asarray(z, dtype=float).reshape(18)
        u = np.asarray(u, dtype=float).reshape(4)

        n = z.size
        m = u.size
        B = np.zeros((n, m))
        f0 = self.f_continuous(z, u)

        for j in range(m):
            up = u.copy()
            up[j] += eps
            fj = self.f_continuous(z, up)
            B[:, j] = (fj - f0) / eps

        return B

    # ---------- ESO step ----------
    def step(self, y_meas, u):
        """
        One ESO update.
        y_meas: [x, y, z, roll, pitch, yaw]
        u: [T, tau_x, tau_y, tau_z]
        """
        Ts = self.Ts

        y_meas = np.asarray(y_meas, dtype=float).reshape(6)
        u = np.asarray(u, dtype=float).reshape(4)

        # 1) Nonlinear prediction
        f = self.f_continuous(self.z, u)
        z_pred = self.z + Ts * f

        # 2) Output prediction
        y_pred = np.array([
            z_pred[0], z_pred[1], z_pred[2],
            z_pred[6], z_pred[7], z_pred[8],
        ])

        # 3) Innovation
        r = y_meas - y_pred

        # 4) Correction
        # MODIFY: L must be tuned correctly, otherwise ESO will diverge
        self.z = z_pred + self.L @ r

        return self.z

    # ---------- optional helper ----------
    def initialize_from_measurement(self, y_meas):
        """
        Initialize ESO from sensor measurement.
        # MODIFY: Recommended to call once at startup
        """
        y_meas = np.asarray(y_meas, dtype=float).reshape(6)
        x, y, z, roll, pitch, yaw = y_meas

        self.z[:] = 0.0
        self.z[0:3] = [x, y, z]
        self.z[6:9] = [roll, pitch, yaw]

        return self.z
