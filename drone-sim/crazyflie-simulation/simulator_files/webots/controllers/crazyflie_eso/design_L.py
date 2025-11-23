import numpy as np
from scipy.signal import place_poles

def compute_L(eso, mass):
    """
    Compute observer gain L (18x6) for the full-state ESO using
    the correct 18x18 linearization and disturbance injection.
    """

    # ------------------------------------------------------------
    # 1) Linearize at hover equilibrium
    # ------------------------------------------------------------
    z_eq = np.zeros(18)
    u_eq = np.array([mass * 9.81, 0.0, 0.0, 0.0])

    # A is 18x18 for the full ESO state
    A, _ = eso.linearize_continuous(z_eq, u_eq)

    # Make a copy that we will modify
    A_mod = A.copy()

    # ------------------------------------------------------------
    # 2) Inject disturbance coupling into A_mod
    # ------------------------------------------------------------
    # State layout in ESO:
    #   0:3   -> p       (x, y, z)
    #   3:6   -> v       (vx, vy, vz)
    #   6:9   -> euler   (roll, pitch, yaw)
    #   9:12  -> omega   (p, q, r)
    #   12:15 -> d_f     (disturbance force)
    #   15:18 -> d_tau   (disturbance torque)

    # d_f enters v_dot
    A_mod[3:6, 12:15] = np.eye(3)

    # d_tau enters omega_dot via J_inv
    A_mod[9:12, 15:18] = eso.J_inv

    # ------------------------------------------------------------
    # 3) Measurement matrix C (6x18)
    # ------------------------------------------------------------
    C = np.zeros((6, 18))
    C[0, 0] = 1.0  # x
    C[1, 1] = 1.0  # y
    C[2, 2] = 1.0  # z
    C[3, 6] = 1.0  # roll
    C[4, 7] = 1.0  # pitch
    C[5, 8] = 1.0  # yaw

    # ------------------------------------------------------------
    # 4) Desired observer poles (tunable)
    # ------------------------------------------------------------
    poles = np.array([
        -12, -12, -12,    # p
        -16, -16, -16,    # v
        -25, -25, -25,    # euler
        -30, -30, -30,    # omega
        -3,  -3,  -3,     # d_f
        -3,  -3,  -3,     # d_tau
    ])

    # ------------------------------------------------------------
    # 5) Place poles on the dual system to get L
    # ------------------------------------------------------------
    sol = place_poles(A_mod.T, C.T, poles)
    L = sol.gain_matrix.T  # 18x6

    return L
