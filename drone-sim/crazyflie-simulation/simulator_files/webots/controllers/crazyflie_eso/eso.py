import numpy as np

class ReducedESO:
    """
    9-state Extended State Observer for position, velocity, and force disturbances.
    
    Uses IMU attitude measurements directly (not estimated).
    
    State z (9×1):
        z = [ x, y, z,           # position (world frame)
              vx, vy, vz,         # velocity (world frame)
              dx, dy, dz ]        # force disturbance (world frame, N)
    
    Measurements y (3×1):
        y = [ x, y, z ]          # GPS position
    
    Inputs:
        u = T (scalar thrust, N)
        roll, pitch, yaw (from IMU)
    """

    def __init__(self, Ts, L, m=0.027, g=9.81):
        self.Ts = Ts
        self.m = float(m)
        self.g = float(g)

        # ESO gain (9×3)
        self.L = np.array(L, dtype=float).reshape(9, 3)
        self.z = np.zeros(9)

    @staticmethod
    def _R(roll, pitch, yaw):
        """Rotation matrix body->world (ZYX Euler convention)"""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        return np.array([
            [cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy],
            [cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy],
            [-sp,   sr*cp,            cr*cp]
        ])

    def f_continuous(self, z, u, roll, pitch, yaw):
        """
        Continuous dynamics with attitude passed in from IMU.
        
        Args:
            z: state [x,y,z, vx,vy,vz, dx,dy,dz]
            u: scalar thrust T (in Newtons)
            roll, pitch, yaw: measured from IMU (radians)
        """
        z = np.asarray(z).reshape(9)
        T = float(u)

        pos = z[0:3]
        vel = z[3:6]
        d_f = z[6:9]

        # Rotation matrix using measured attitude
        R = self._R(roll, pitch, yaw)

        # Position dynamics
        dp = vel

        # Velocity dynamics
        # a = R·[0,0,T]/m - [0,0,g] + d_f
        thrust_body = np.array([0, 0, T])
        thrust_world = R @ thrust_body
        dv = -np.array([0, 0, self.g]) + thrust_world / self.m + d_f

        # Disturbance dynamics (constant/slowly-varying)
        dd_f = np.zeros(3)

        return np.concatenate([dp, dv, dd_f])

    def jacobian_x(self, z, u, roll, pitch, yaw):
        """Jacobian w.r.t. state using central differences."""
        z = np.asarray(z)
        n = len(z)
        A = np.zeros((n, n))
        
        for i in range(n):
            eps = max(1e-6, 1e-5 * abs(z[i]))
            
            zp = z.copy()
            zm = z.copy()
            zp[i] += eps
            zm[i] -= eps
            
            fp = self.f_continuous(zp, u, roll, pitch, yaw)
            fm = self.f_continuous(zm, u, roll, pitch, yaw)
            A[:, i] = (fp - fm) / (2 * eps)
        
        return A

    def jacobian_u(self, z, u, roll, pitch, yaw):
        """Jacobian w.r.t. thrust input."""
        z = np.asarray(z)
        B = np.zeros((len(z), 1))
        
        eps = max(1e-6, 1e-5 * abs(u)) if abs(u) > 1e-8 else 1e-6
        
        fp = self.f_continuous(z, u + eps, roll, pitch, yaw)
        fm = self.f_continuous(z, u - eps, roll, pitch, yaw)
        B[:, 0] = (fp - fm) / (2 * eps)
        
        return B

    def linearize_continuous(self, z, u, roll, pitch, yaw):
        """Linearize dynamics around current state."""
        A = self.jacobian_x(z, u, roll, pitch, yaw)
        B = self.jacobian_u(z, u, roll, pitch, yaw)
        return A, B

    def step(self, y_pos, u_thrust, roll, pitch, yaw):
        """
        Discrete-time predictor-corrector ESO update.
        
        Args:
            y_pos: measured position [x, y, z] from GPS
            u_thrust: scalar thrust T (Newtons)
            roll, pitch, yaw: measured attitude from IMU (radians)
        """
        Ts = self.Ts
        
        # PREDICT
        z_pred = self.z + Ts * self.f_continuous(self.z, u_thrust, roll, pitch, yaw)
        
        # INNOVATION (from predicted position)
        y_pred = z_pred[0:3]  # predicted position
        r = y_pos - y_pred
        
        # CORRECT
        self.z = z_pred + self.L @ r
        
        return self.z

    def initialize_from_measurement(self, y_pos):
        """Initialize ESO from first GPS measurement."""
        y = np.asarray(y_pos)
        self.z[:] = 0.0
        self.z[0:3] = y  # position
        # velocity and disturbances initialize to zero
        return self.z