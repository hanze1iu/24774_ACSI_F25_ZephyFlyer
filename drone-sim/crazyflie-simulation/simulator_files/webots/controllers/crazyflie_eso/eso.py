import numpy as np

class FullStateESO:
    """
    Full nonlinear 18-state Extended State Observer for Crazyflie-style quadrotor.

    State z (18×1):
        z = [ p(3);
              v(3);
              euler(3);
              omega(3);
              d_f(3);
              d_tau(3) ]

    Inputs u = [T, τx, τy, τz]
    """

    def __init__(self, Ts, L, m=0.027, J=None, g=9.81):
        self.Ts = Ts
        self.m = float(m)
        self.g = float(g)

        if J is None:
            J = np.diag([1.395e-5, 1.436e-5, 2.173e-5])
        self.J = np.array(J, dtype=float)
        self.J_inv = np.linalg.inv(self.J)

        # ESO gain (18×6)
        self.L = np.array(L, dtype=float).reshape(18, 6)

        # observer state
        self.z = np.zeros(18)

    # ---------------------------------------------
    # Rotation matrix R(eta) body→world (ZYX)
    # ---------------------------------------------
    @staticmethod
    def _R(eta):
        phi, theta, psi = eta
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth, sth = np.cos(theta), np.sin(theta)
        cpsi, spsi = np.cos(psi), np.sin(psi)

        return np.array([
            [cth*cpsi,  cth*spsi, -sth],
            [sphi*sth*cpsi - cphi*spsi,
             sphi*sth*spsi + cphi*cpsi,
             sphi*cth],
            [cphi*sth*cpsi + sphi*spsi,
             cphi*sth*spsi - sphi*cpsi,
             cphi*cth]
        ])

    # ---------------------------------------------
    # Euler kinematics
    # ---------------------------------------------
    @staticmethod
    def _E(eta):
        phi, theta, _ = eta
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth, sth = np.cos(theta), np.sin(theta)

        return np.array([
            [1.0,  sphi*np.tan(theta),  cphi*np.tan(theta)],
            [0.0,  cphi,               -sphi],
            [0.0,  sphi/cth,            cphi/cth],
        ])

    # ---------------------------------------------
    # Continuous dynamics f(z,u)
    # ---------------------------------------------
    def f_continuous(self, z, u):
        z = np.asarray(z).reshape(18)
        u = np.asarray(u).reshape(4)

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

        # p_dot = v
        dp = v

        # v_dot
        thrust_world = R @ np.array([0, 0, T])
        dv = (-np.array([0, 0, self.g]) +
              thrust_world / self.m +
              d_f)

        # eta_dot
        deta = E @ omega

        # omega_dot
        domega = self.J_inv @ (tau -
                               np.cross(omega, self.J @ omega) +
                               d_tau)

        # disturbance terms (integrators)
        dd_f = np.zeros(3)
        dd_tau = np.zeros(3)

        return np.concatenate([dp, dv, deta, domega, dd_f, dd_tau])

    # ---------------------------------------------
    # Jacobians via finite difference
    # ---------------------------------------------
    def jacobian_x(self, z, u, eps=1e-5):
        z = np.asarray(z)
        u = np.asarray(u)
        f0 = self.f_continuous(z, u)

        n = len(z)
        A = np.zeros((n, n))

        for i in range(n):
            zp = z.copy()
            zp[i] += eps
            fi = self.f_continuous(zp, u)
            A[:, i] = (fi - f0) / eps

        return A

    def jacobian_u(self, z, u, eps=1e-5):
        z = np.asarray(z)
        u = np.asarray(u)
        f0 = self.f_continuous(z, u)

        m = len(u)
        B = np.zeros((len(z), m))

        for j in range(m):
            up = u.copy()
            up[j] += eps
            fj = self.f_continuous(z, up)
            B[:, j] = (fj - f0) / eps

        return B

    # ---------------------------------------------
    # Continuous linearization A,B
    # ---------------------------------------------
    def linearize_continuous(self, z, u):
        A = self.jacobian_x(z, u)
        B = self.jacobian_u(z, u)
        return A, B

    # ---------------------------------------------
    # Discrete linearization Ad, Bd
    # ---------------------------------------------
    def linearize_discrete(self, z, u):
        A, B = self.linearize_continuous(z, u)
        Ts = self.Ts
        Ad = np.eye(18) + Ts * A
        Bd = Ts * B
        return Ad, Bd

    # ---------------------------------------------
    # ESO update
    # ---------------------------------------------
    def step(self, y_meas, u):
        Ts = self.Ts

        y_pred = np.array([
            self.z[0], self.z[1], self.z[2],
            self.z[6], self.z[7], self.z[8]
        ])

        r = y_meas - y_pred

        z_pred = self.z + Ts * self.f_continuous(self.z, u)
        self.z = z_pred + self.L @ r
        return self.z

    def initialize_from_measurement(self, y_meas):
        y = np.asarray(y_meas)
        self.z[:] = 0.0
        self.z[0:3] = y[0:3]
        self.z[6:9] = y[3:6]
        return self.z
