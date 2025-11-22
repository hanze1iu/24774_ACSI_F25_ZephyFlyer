"""
Crazyflie outer-loop controller with Full-State ESO structure.
"""

from controller import Robot, Keyboard
import numpy as np
import sys
sys.path.append('../../../../controllers_shared/python_based')

from pid_controller import pid_velocity_fixed_height_controller
from ESO import FullStateESO

# =========================
# Simulation setup
# =========================
robot = Robot()
timestep = int(robot.getBasicTimeStep())
Ts = timestep / 1000.0

# =========================
# Full-State ESO initialization
# =========================

m = 0.027  # MODIFY: use correct CF mass
J = np.diag([1.4e-5, 1.4e-5, 2.17e-5])  # MODIFY: correct inertia matrix
L = np.zeros((18, 6))  # MODIFY: insert observer gain after design

eso = FullStateESO(Ts, m, J, L)


# in your Webots loop:
y_meas = np.array([pos[0], pos[1], pos[2], roll, pitch, yaw])
u_cmd  = np.array([T_cmd, tau_x_cmd, tau_y_cmd, tau_z_cmd])
z_hat  = eso.step(y_meas, u_cmd)


# =========================
# Simulation setup
# =========================
robot = Robot()
timestep = int(robot.getBasicTimeStep())
Ts = timestep / 1000.0

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
for m in motors:
    m.setPosition(float('inf'))
    m.setVelocity(0.0)

# =========================
# WAIT HERE for valid GPS
# =========================
print("Waiting for GPS initialization...")
while robot.step(timestep) != -1:
    pos0 = np.array(gps.getValues())
    if np.isfinite(pos0).all() and np.linalg.norm(pos0) > 1e-6:
        break
print(f"GPS initialized at position {pos0}")

# =========================
# Helper functions
# =========================
def body_velocity_from_global(vx_g, vy_g, yaw):
    """Rotate global velocities into body frame."""
    cosy, siny = np.cos(yaw), np.sin(yaw)
    vx = vx_g * cosy + vy_g * siny
    vy = -vx_g * siny + vy_g * cosy
    return vx, vy


# =========================
# Inner PID controller
# =========================
PID_CF = pid_velocity_fixed_height_controller()
height_desired = 0.5

# Trajectory setup
radius = 1.0
omega = 0.2
z_ref = 0.5

print("\n>>> Crazyflie controller running (circular path test, no xyz ESO) <<<")
prev_pos = pos0.copy()
past_time = robot.getTime()

# Simple outer-loop PD gains in global frame
Kp_pos = 1.5
Kd_pos = 0.4

# MAIN LOOP
while robot.step(timestep) != -1:

    t = robot.getTime()
    dt = max(t - past_time, 1e-3)
    past_time = t

    # ------------------------------------------------------
    # 1) Sensor readings
    # ------------------------------------------------------
    pos = np.array(gps.getValues())

    roll, pitch, yaw = imu.getRollPitchYaw()
    yaw_rate = gyro.getValues()[2]

    # ------------------------------------------------------
    # 2) Run ESO
    # ------------------------------------------------------
    y_meas = np.array([
        pos[0], pos[1], pos[2],   # x y z
        roll, pitch, yaw          # euler
    ])

    # MODIFY: Replace these with actual thrust & torques once you compute them
    T_cmd = 0.0     # MODIFY
    tau_x_cmd = 0.0 # MODIFY
    tau_y_cmd = 0.0 # MODIFY
    tau_z_cmd = 0.0 # MODIFY

    u_cmd = np.array([T_cmd, tau_x_cmd, tau_y_cmd, tau_z_cmd])

    z_hat = eso.step(y_meas, u_cmd)

    # Unpack ESO estimates
    eso_p     = z_hat[0:3]
    eso_v     = z_hat[3:6]
    eso_euler = z_hat[6:9]
    eso_omega = z_hat[9:12]

    # Example convenience variables:
    eso_roll, eso_pitch, eso_yaw = eso_euler
    eso_vx, eso_vy, eso_vz = eso_v

    # ------------------------------------------------------
    # 3) Outer-loop (position → desired velocity)
    # ------------------------------------------------------
    pos_err = target - pos
    vx_des_global = Kp_pos * pos_err[0] - Kd_pos * eso_vx
    vy_des_global = Kp_pos * pos_err[1] - Kd_pos * eso_vy

    vx_des_global = np.clip(vx_des_global, -0.6, 0.6)
    vy_des_global = np.clip(vy_des_global, -0.6, 0.6)

    # ------------------------------------------------------
    # 4) Convert desired global velocity to body-frame
    # ------------------------------------------------------
    v_body_x, v_body_y = body_velocity_from_global(
        vx_des_global, vy_des_global, eso_yaw
    )

    forward_desired  = v_body_x
    sideways_desired = v_body_y
    yaw_desired      = 0.0
    height_desired   = z_ref

    # ------------------------------------------------------
    # 5) Inner PID (Bitcraze official)
    # ------------------------------------------------------
    altitude = pos[2]
    v_x_body = eso_vx
    v_y_body = eso_vy

    motor_power = PID_CF.pid(
        dt,
        forward_desired, sideways_desired,
        yaw_desired, height_desired,
        eso_roll, eso_pitch, eso_omega[2],
        altitude, v_x_body, v_y_body
    )

    # ------------------------------------------------------
    # 6) Apply motor commands
    # ------------------------------------------------------
    m1_motor.setVelocity(-motor_power[0])
    m2_motor.setVelocity( motor_power[1])
    m3_motor.setVelocity(-motor_power[2])
    m4_motor.setVelocity( motor_power[3])


    # --- Print every 0.5 s ---
    if int(t*10) % 5 == 0:
        print(
            f"t={t:4.1f} | Ref=({x_ref:+.2f},{y_ref:+.2f}) "
            f"| Pos=({pos[0]:+.2f},{pos[1]:+.2f},{pos[2]:+.2f}) "
            f"| v_des_g=({vx_des_global:+.2f},{vy_des_global:+.2f})"
        )
