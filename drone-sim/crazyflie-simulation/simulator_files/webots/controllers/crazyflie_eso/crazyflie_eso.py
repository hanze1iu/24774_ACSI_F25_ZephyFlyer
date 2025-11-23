"""
Crazyflie outer-loop controller with Full-State ESO structure.
"""

from controller import Robot, Keyboard
import numpy as np
import sys
sys.path.append('../../../../controllers_shared/python_based')

from pid_controller import pid_velocity_fixed_height_controller
from eso import FullStateESO
from design_L import compute_L
# =========================
# Simulation setup
# =========================
robot = Robot()
timestep = int(robot.getBasicTimeStep())
Ts = timestep / 1000.0

# =========================
# Full-State ESO initialization
# =========================
m = 0.027
J = np.diag([1.4e-5, 1.4e-5, 2.17e-5])

# 1) Create ESO with zero gains first
eso = FullStateESO(Ts, np.zeros((18,6)), m, J)

# 2) Design L using correct system model
L = compute_L(eso, m)

# 3) Assign it (do NOT recreate ESO)
eso.L = L



# =========================
# Devices
# =========================
keyboard = Keyboard()
keyboard.enable(timestep)

imu = robot.getDevice("inertial_unit"); imu.enable(timestep)
gps = robot.getDevice("gps"); gps.enable(timestep)
gyro = robot.getDevice("gyro"); gyro.enable(timestep)

motors = [
    robot.getDevice("m1_motor"),
    robot.getDevice("m2_motor"),
    robot.getDevice("m3_motor"),
    robot.getDevice("m4_motor"),
]
for motor in motors:
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)


# =========================
# Wait for GPS
# =========================
print("Waiting for GPS initialization...")
while robot.step(timestep) != -1:
    pos0 = np.array(gps.getValues())
    if np.isfinite(pos0).all() and np.linalg.norm(pos0) > 1e-6:
        break
print(f"GPS initialized at position {pos0}")

# =========================
# Helper
# =========================
def body_velocity_from_global(vx_g, vy_g, yaw):
    cosy, siny = np.cos(yaw), np.sin(yaw)
    vx = vx_g * cosy + vy_g * siny
    vy = -vx_g * siny + vy_g * cosy
    return vx, vy

# =========================
# Inner PID controller
# =========================
PID_CF = pid_velocity_fixed_height_controller()

# =========================
# Desired hover control input
# =========================
T_cmd = m * 9.81
tau_x_cmd = 0
tau_y_cmd = 0
tau_z_cmd = 0



# =========================
# Target position (circle center)
# =========================
target = np.array([0.0, 0.0, 0.5])   # YOU MUST DEFINE THIS

# =========================
# PD gains
# =========================
Kp_pos = 1.5
Kd_pos = 0.4

past_time = robot.getTime()

# =========================
# MAIN LOOP
# =========================
while robot.step(timestep) != -1:

    t = robot.getTime()
    dt = max(t - past_time, 1e-3)
    past_time = t

    # ------------------------------
    # 1) Read sensors
    # ------------------------------
    pos = np.array(gps.getValues())
    roll, pitch, yaw = imu.getRollPitchYaw()
    yaw_rate = gyro.getValues()[2]

    # ------------------------------
    # 2) ESO update
    # ------------------------------
    y_meas = np.array([pos[0], pos[1], pos[2],
                       roll, pitch, yaw])

    u_cmd = np.array([T_cmd, tau_x_cmd, tau_y_cmd, tau_z_cmd])
    z_hat = eso.step(y_meas, u_cmd)

    eso_p = z_hat[0:3]
    eso_v = z_hat[3:6]
    eso_euler = z_hat[6:9]
    eso_omega = z_hat[9:12]
    # Disturbance estimates
    eso_d_f   = z_hat[12:15]   # [Nx, Ny, Nz] in world frame
    eso_d_tau = z_hat[15:18]   # [τx, τy, τz] in body frame

    eso_roll, eso_pitch, eso_yaw = eso_euler
    eso_vx, eso_vy, eso_vz = eso_v
    
    
    # ------------------------------
    # 2.5) Linearization
    # ------------------------------
    A, B = eso.linearize_continuous(z_hat, u_cmd)
    
    # ------------------------------
    # 3) Position → velocity command
    # ------------------------------
    pos_err = target - pos
    vx_des_global = Kp_pos * pos_err[0] - Kd_pos * eso_vx
    vy_des_global = Kp_pos * pos_err[1] - Kd_pos * eso_vy

    vx_des_global = np.clip(vx_des_global, -0.6, 0.6)
    vy_des_global = np.clip(vy_des_global, -0.6, 0.6)

    v_body_x, v_body_y = body_velocity_from_global(
        vx_des_global, vy_des_global, eso_yaw
    )

    forward_desired = v_body_x
    sideways_desired = v_body_y
    yaw_desired = 0.0
    height_desired = target[2]

    # ------------------------------
    # 4) Inner PID
    # ------------------------------
    altitude = pos[2]

    v_x_body = eso_vx
    v_y_body = eso_vy

    motor_power = PID_CF.pid(
        dt,
        forward_desired, sideways_desired,
        yaw_desired, height_desired,
        eso_roll, eso_pitch, yaw_rate,   # use gyro z-rate
        altitude, v_x_body, v_y_body
    )

    # ------------------------------
    # 5) Apply to motors
    # ------------------------------
    motors[0].setVelocity(-motor_power[0])
    motors[1].setVelocity( motor_power[1])
    motors[2].setVelocity(-motor_power[2])
    motors[3].setVelocity( motor_power[3])

    # Debug print
    if int(t * 10) % 10 == 0:
        print(f"t={t:.2f} pos={pos} v_des=({vx_des_global:.2f},{vy_des_global:.2f})")
    
        print(
            f"[ESO] pos=({eso_p[0]:+.2f},{eso_p[1]:+.2f},{eso_p[2]:+.2f}) "
            f"vel=({eso_vx:+.2f},{eso_vy:+.2f},{eso_vz:+.2f}) "
            f"att=(rpy={eso_roll:+.2f},{eso_pitch:+.2f},{eso_yaw:+.2f})\n"
            f"      d_f =({eso_d_f[0]:+.3f},{eso_d_f[1]:+.3f},{eso_d_f[2]:+.3f}) "
            f"d_tau=({eso_d_tau[0]:+.4f},{eso_d_tau[1]:+.4f},{eso_d_tau[2]:+.4f})"
        )
        