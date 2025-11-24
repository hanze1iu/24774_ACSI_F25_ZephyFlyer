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
# Sim
# =========================
robot = Robot()
timestep = int(robot.getBasicTimeStep())
Ts = timestep / 1000.0

# =========================
# ESO initialization
# =========================
m = 0.027 # AMY: Need to hand measure this
J = np.diag([1.4e-5, 1.4e-5, 2.17e-5])

# 1) Create ESO with zero gains first
eso = FullStateESO(Ts, np.zeros((18,6)), m, J)

# 2) Design L using system model
L = compute_L(eso, m)

# 3) Assign
eso.L = L

# =========================
# Read devices
# =========================
keyboard = Keyboard()
keyboard.enable(timestep)

imu = robot.getDevice("inertial_unit")
imu.enable(timestep)
gps = robot.getDevice("gps")
gps.enable(timestep)
gyro = robot.getDevice("gyro")
gyro.enable(timestep)

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
# Wait for GPS and IMU initialization
# =========================
print("Waiting for sensor initialization...")
initialization_steps = 0
while robot.step(timestep) != -1:
    pos0 = np.array(gps.getValues())
    rpy0 = imu.getRollPitchYaw()
    
    if np.isfinite(pos0).all() and np.isfinite(rpy0).all():
        if np.linalg.norm(pos0) > 1e-6:
            break
    
    initialization_steps += 1
    if initialization_steps > 100:
        print("Warning: Sensor initialization taking longer than expected")
        break

print(f"GPS initialized at position {pos0}")

# =========================
# Initialize ESO from first measurement
# =========================
roll0, pitch0, yaw0 = imu.getRollPitchYaw()
y_init = np.array([pos0[0], pos0[1], pos0[2], roll0, pitch0, yaw0])
eso.initialize_from_measurement(y_init)
print(f"ESO initialized with state: pos={pos0}, rpy=({roll0:.3f},{pitch0:.3f},{yaw0:.3f})")

# =========================
# Helper
# =========================
def body_velocity_from_global(vx_g, vy_g, yaw):
    """Transform global velocities to body frame."""
    cosy, siny = np.cos(yaw), np.sin(yaw)
    vx = vx_g * cosy + vy_g * siny
    vy = -vx_g * siny + vy_g * cosy
    return vx, vy

def global_velocity_from_body(vx_b, vy_b, yaw):
    """Transform body velocities to global frame."""
    cosy, siny = np.cos(yaw), np.sin(yaw)
    vx_g = vx_b * cosy - vy_b * siny
    vy_g = vx_b * siny + vy_b * cosy
    return vx_g, vy_g

# =========================
# Inner PID controller
# =========================
PID_CF = pid_velocity_fixed_height_controller()

# =========================
# Desired hover control input
# =========================
T_cmd = m * 9.81
tau_x_cmd = 0.0
tau_y_cmd = 0.0
tau_z_cmd = 0.0

# =========================
# Target position
# =========================
target = np.array([0.0, 0.0, 0.5])

# =========================
# PD gains
# =========================
Kp_pos = 2.0   # 1.5
Kd_pos = 0.8   # 0.4

past_time = robot.getTime()

# =========================
# Velocity limits
# =========================
VEL_LIMIT = 0.5  # m/s

# =========================
# Print timing control
# =========================
last_print_time = 0.0
PRINT_INTERVAL = 0.1  # seconds

# =========================
# MAIN LOOP
# =========================
print("Starting main control loop...")
loop_count = 0

while robot.step(timestep) != -1:

    t = robot.getTime()
    dt = t - past_time
    past_time = t
    loop_count += 1

    # ------------------------------
    # 1) Read sensors
    # ------------------------------
    pos = np.array(gps.getValues())
    roll, pitch, yaw = imu.getRollPitchYaw()  # Direct from IMU
    omega_body = np.array(gyro.getValues())
    yaw_rate = omega_body[2]

    # Sanity check for sensor readings
    if not (np.isfinite(pos).all() and np.isfinite([roll, pitch, yaw]).all()):
        print(f"Warning: Invalid sensor readings at t={t:.2f}")
        continue

    # ------------------------------
    # 2) ESO update
    # ------------------------------
    y_meas = np.array([pos[0], pos[1], pos[2],
                    roll, pitch, yaw])  # Feed IMU to ESO

    u_cmd = np.array([T_cmd, tau_x_cmd, tau_y_cmd, tau_z_cmd])
    z_hat = eso.step(y_meas, u_cmd)

    # Extract estimated states
    eso_p = z_hat[0:3]
    eso_v = z_hat[3:6]      # Velocity in WORLD frame
    eso_euler = z_hat[6:9]  # ESO attitude (NOT USED for control!)
    eso_omega = z_hat[9:12]
    eso_d_f   = z_hat[12:15]
    eso_d_tau = z_hat[15:18]

    eso_roll, eso_pitch, eso_yaw = eso_euler  # Only for logging
    eso_vx_world, eso_vy_world, eso_vz = eso_v

    # ------------------------------
    # 3) Position → velocity command (in WORLD frame)
    # ------------------------------
    pos_err = target - eso_p

    vx_des_world = Kp_pos * pos_err[0] + Kd_pos * (-eso_vx_world)
    vy_des_world = Kp_pos * pos_err[1] + Kd_pos * (-eso_vy_world)
    vz_des_world = Kp_pos * pos_err[2] + Kd_pos * (-eso_vz)

    vx_des_world = np.clip(vx_des_world, -VEL_LIMIT, VEL_LIMIT)
    vy_des_world = np.clip(vy_des_world, -VEL_LIMIT, VEL_LIMIT)
    vz_des_world = np.clip(vz_des_world, -VEL_LIMIT, VEL_LIMIT)

    # ------------------------------
    # 4) Transform to body frame for PID controller
    # ------------------------------
    # CRITICAL: Use IMU yaw for transformation, not ESO yaw
    v_body_x_des, v_body_y_des = body_velocity_from_global(
        vx_des_world, vy_des_world, yaw  # ← Use IMU yaw!
    )

    forward_desired = v_body_x_des
    sideways_desired = v_body_y_des
    yaw_desired = 0.0
    height_desired = target[2]

    # ------------------------------
    # 5) Inner PID controller
    # ------------------------------
    # CRITICAL: Convert ESO world-frame velocities using IMU yaw
    eso_vx_body, eso_vy_body = body_velocity_from_global(
        eso_vx_world, eso_vy_world, yaw  # ← Use IMU yaw!
    )

    # Use IMU attitude directly for control
    motor_power = PID_CF.pid(
        dt,
        forward_desired,      # Body-frame x velocity command
        sideways_desired,     # Body-frame y velocity command
        yaw_desired,          # Yaw angle command
        height_desired,       # Altitude command
        roll,                 # ← IMU (CORRECT!)
        pitch,                # ← IMU (CORRECT!)
        yaw_rate,             # ← Gyro (CORRECT!)
        eso_p[2],             # ESO altitude (OK)
        eso_vx_body,          # ESO velocity (OK)
        eso_vy_body           # ESO velocity (OK)
    )

    # ------------------------------
    # 6) Safety checks and motor application
    # ------------------------------
    # Clamp motor commands
    motor_power = np.clip(motor_power, 0, 600)

    # Sanity check
    if np.any(np.isnan(motor_power)) or np.any(np.isinf(motor_power)):
        print(f"ERROR: Invalid motor commands at t={t:.2f}")
        motor_power = np.array([100, 100, 100, 100])

    # Attitude safety check
    if abs(roll) > np.pi/3 or abs(pitch) > np.pi/3:  # > 60°
        print(f"WARNING: Extreme attitude at t={t:.2f}, roll={roll*180/np.pi:.1f}°, pitch={pitch*180/np.pi:.1f}°")
        # Consider emergency landing

    # Apply to motors
    motors[0].setVelocity(-motor_power[0])
    motors[1].setVelocity( motor_power[1])
    motors[2].setVelocity(-motor_power[2])
    motors[3].setVelocity( motor_power[3])

    # ------------------------------
    # Prints
    # ------------------------------
    if (t - last_print_time) >= PRINT_INTERVAL:
        last_print_time = t
        
        pos_error_norm = np.linalg.norm(pos_err)
        vel_world_norm = np.linalg.norm([eso_vx_world, eso_vy_world, eso_vz])
        
        print(f"\n{'='*60}")
        print(f"Time: {t:.2f}s")
        print(f"Position: GPS=({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}), "
              f"ESO=({eso_p[0]:+.3f}, {eso_p[1]:+.3f}, {eso_p[2]:+.3f})")
        print(f"Error: ({pos_err[0]:+.3f}, {pos_err[1]:+.3f}, {pos_err[2]:+.3f}), "
              f"norm={pos_error_norm:.3f}m")
        print(f"Velocity (world): ({eso_vx_world:+.3f}, {eso_vy_world:+.3f}, {eso_vz:+.3f}), "
              f"norm={vel_world_norm:.3f}m/s")
        print(f"Velocity (body):  ({eso_vx_body:+.3f}, {eso_vy_body:+.3f})")
        print(f"Attitude: roll={eso_roll*180/np.pi:+.1f}°, "
              f"pitch={eso_pitch*180/np.pi:+.1f}°, "
              f"yaw={eso_yaw*180/np.pi:+.1f}°")
        print(f"Disturbances: F=({eso_d_f[0]:+.4f}, {eso_d_f[1]:+.4f}, {eso_d_f[2]:+.4f})N, "
              f"τ=({eso_d_tau[0]:+.5f}, {eso_d_tau[1]:+.5f}, {eso_d_tau[2]:+.5f})N·m")
        print(f"Motor power: [{motor_power[0]:.1f}, {motor_power[1]:.1f}, "
              f"{motor_power[2]:.1f}, {motor_power[3]:.1f}]")