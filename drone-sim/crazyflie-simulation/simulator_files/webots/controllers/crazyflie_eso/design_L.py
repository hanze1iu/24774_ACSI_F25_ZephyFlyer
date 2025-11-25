import numpy as np
from scipy.linalg import solve_continuous_are

def compute_L(eso, mass):
    """
    Observer gain design for 9-state ESO (position + velocity + force disturbances).
    Uses LQR approach with Q/R from controller.
    """
    
    print("\n" + "="*60)
    print("9-STATE ESO GAIN DESIGN")
    print("="*60)
    
    # ------------------------------------------------------------
    # 1) Linearize at hover
    # ------------------------------------------------------------
    z_eq = np.zeros(9)
    u_eq = mass * 9.81  # Hover thrust
    roll_eq = 0.0
    pitch_eq = 0.0
    yaw_eq = 0.0
    
    A, B = eso.linearize_continuous(z_eq, u_eq, roll_eq, pitch_eq, yaw_eq)
    
    # Force disturbance coupling
    A_mod = A.copy()
    A_mod[6:9, 6:9] = 0  # Disturbances are integrators
    A_mod[3:6, 6:9] = np.eye(3)  # d_f directly affects acceleration
    
    # Measurement matrix (position only)
    C = np.zeros((3, 9))
    C[0:3, 0:3] = np.eye(3)
    
    # Check observability
    n = A_mod.shape[0]
    O = C
    for i in range(1, n):
        O = np.vstack([O, C @ np.linalg.matrix_power(A_mod, i)])
    rank = np.linalg.matrix_rank(O)
    print(f"Observability: rank={rank}/{n}")
    
    # ------------------------------------------------------------
    # 2) Controller Q/R
    # ------------------------------------------------------------
    Q_controller = np.diag([
        50, 50, 20,     # roll, pitch, yaw
        10, 10, 5,      # p, q, r
        30, 30, 60,     # x, y, z
        10, 10, 20      # vx, vy, vz
    ])
    
    # ============================================
    # MANUAL TUNING KNOBS
    # ============================================
    observer_speedup = 1.0
    position_scale = 0.2
    velocity_scale = 0.3
    force_dist_scale = 0.1
    gps_noise = 0.05
    # ============================================
    
    print(f"Tuning: speedup={observer_speedup}, vel_scale={velocity_scale}")
    
    # Map to observer Q
    Q_ctrl_diag = np.diag(Q_controller)
    Q_observer = np.zeros(9)
    
    # Position (indices 0:3)
    Q_observer[0:3] = Q_ctrl_diag[6:9] * observer_speedup * position_scale
    
    # Velocity (indices 3:6)
    Q_observer[3:6] = Q_ctrl_diag[9:12] * observer_speedup * velocity_scale
    
    # Force disturbances (indices 6:9)
    Q_observer[6:9] = Q_ctrl_diag[9:12] * force_dist_scale
    
    Q_observer = np.diag(Q_observer)
    
    # Measurement noise
    R_observer = np.diag([gps_noise, gps_noise, gps_noise])
    
    print(f"Q range: [{np.min(np.diag(Q_observer)):.2f}, {np.max(np.diag(Q_observer)):.2f}]")
    print(f"R range: [{np.min(np.diag(R_observer)):.4f}, {np.max(np.diag(R_observer)):.4f}]")
    
    # ------------------------------------------------------------
    # 3) Solve Riccati
    # ------------------------------------------------------------
    try:
        P = solve_continuous_are(A_mod.T, C.T, Q_observer, R_observer)
        L_continuous = P @ C.T @ np.linalg.inv(R_observer)
        L_discrete = eso.Ts * L_continuous
        
        max_gain = np.max(np.abs(L_discrete))
        print(f"\n✓ Observer gain computed")
        print(f"Max discrete gain: {max_gain:.2f}")
        
        # Auto-scale if too high
        TARGET_MAX_GAIN = 40.0
        if max_gain > TARGET_MAX_GAIN:
            print(f"⚠ Reducing gain from {max_gain:.1f} to ~{TARGET_MAX_GAIN}")
            scale_factor = (TARGET_MAX_GAIN / max_gain) ** 2
            Q_observer_scaled = Q_observer * scale_factor
            
            P = solve_continuous_are(A_mod.T, C.T, Q_observer_scaled, R_observer)
            L_continuous = P @ C.T @ np.linalg.inv(R_observer)
            L_discrete = eso.Ts * L_continuous
            max_gain = np.max(np.abs(L_discrete))
            print(f"✓ Scaled max gain: {max_gain:.2f}")
        
        # Stability check
        A_cl = A_mod - L_continuous @ C
        eigs = np.linalg.eigvals(A_cl)
        max_real = np.max(eigs.real)
        print(f"Stability: max eigenvalue real part = {max_real:.2f}")
        
        if max_real >= 0:
            raise ValueError("Unstable!")
        
        print("="*60 + "\n")
        return L_discrete
        
    except Exception as e:
        print(f"✗ Failed: {e}")
        print("Using fallback gains\n")
        
        L = np.zeros((9, 3))
        L[0:3, 0:3] = 2.0 * eso.Ts * np.eye(3)
        L[3:6, 0:3] = 3.0 * eso.Ts * np.eye(3)
        L[6:9, 0:3] = 0.5 * eso.Ts * np.eye(3)
        
        print(f"Fallback max gain: {np.max(np.abs(L)):.2f}")
        print("="*60 + "\n")
        return L