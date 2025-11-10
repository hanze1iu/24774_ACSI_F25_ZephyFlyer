"""
Crazyflie ESO + ADRC outer loop controller.
Uses ESO to estimate states and generate desired body velocities.
"""

from controller import Robot, Keyboard
import numpy as np
import sys
sys.path.append('../../../../controllers_shared/python_based')
from pid_controller import pid_velocity_fixed_height_controller
from eso import ESO

# =========================
# Simulation setup
# =========================
robot = Robot()
timestep = int(robot.getBasicTimeStep())
Ts = timestep / 1000.0

keyboard = Keyboard()
keyboard.enable(timestep)

# === Devices ===
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
# WAIT HERE for valid GPS   <-- INSERTED
# =========================
print("Waiting for GPS initialization...")
while robot.step(timestep) != -1:
    pos0 = np.array(gps.getValues())
    if np.isfinite(pos0).all() and np.linalg.norm(pos0) > 1e-6:
        break
print(f"GPS initialized at position {pos0}")

# =========================
# ESO / ADRC setup
# =========================
b_hat = 1.0
w_c = 3.0
w_o = 4.0
Kp = w_c ** 2
Kd = 2.0 * w_c

eso_x = ESO(Ts, b_hat, w_o)
eso_y = ESO(Ts, b_hat, w_o)
eso_z = ESO(Ts, b_hat, w_o)

# Initialize ESO from the now-valid GPS
eso_x.x1, eso_x.x2, eso_x.d = pos0[0], 0.0, 0.0
eso_y.x1, eso_y.x2, eso_y.d = pos0[1], 0.0, 0.0
eso_z.x1, eso_z.x2, eso_z.d = pos0[2], 0.0, 0.0

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

# Trajectory setup (unchanged)
radius = 1.0
omega = 0.2
z_ref = 0.5

print("\n>>> Crazyflie ESO controller running (circular path test) <<<")
prev_pos = pos0.copy()              # <-- use the valid GPS you just waited for
past_time = robot.getTime()


# MAIN LOOP
while robot.step(timestep) != -1:
    # === (A) Check ESO state before using it ===
    if (not np.isfinite(eso_x.x1) or not np.isfinite(eso_x.x2) or not np.isfinite(eso_x.d)):
        print(f"[PRE-ESO NaN] eso_x had NaN before update! x1={eso_x.x1}, x2={eso_x.x2}, d={eso_x.d}")
        # quick reset
        pos = np.array(gps.getValues())
        eso_x.x1, eso_x.x2, eso_x.d = pos[0], 0.0, 0.0

    # === (B) Time update ===
    t = robot.getTime()
    dt = max(t - past_time, 1e-3)
    past_time = t

    # --- Reference trajectory (circle) ---
    x_ref = radius * np.cos(omega * t)
    y_ref = radius * np.sin(omega * t)
    target = np.array([x_ref, y_ref, z_ref])

    # --- Sensor readings ---
    pos = np.array(gps.getValues())
    roll, pitch, yaw = imu.getRollPitchYaw()
    yaw_rate = gyro.getValues()[2]

    # === (2) GPS validity check goes here ===
    if not np.isfinite(pos).all():
        print(f"[NaN] GPS returned non-finite values at t={t:.2f}: {pos}")
        continue   # skip this loop iteration until GPS is valid

    # --- ESO/ADRC outer loop (raw) ---
    ux_raw = (Kp*(target[0]-eso_x.x1) - Kd*eso_x.x2 - eso_x.d) / b_hat
    uy_raw = (Kp*(target[1]-eso_y.x1) - Kd*eso_y.x2 - eso_y.d) / b_hat
    uz_raw = (Kp*(target[2]-eso_z.x1) - Kd*eso_z.x2 - eso_z.d) / b_hat

    # === (3) Check for NaN before ESO.update ===
    if (not np.isfinite(ux_raw) or not np.isfinite(uy_raw) or not np.isfinite(uz_raw)):
        print(f"[NaN before ESO.update] t={t:.2f} "
              f"ux_raw={ux_raw}, uy_raw={uy_raw}, uz_raw={uz_raw}")
        break  # stop or handle reset here

    # SAFETY: clamp BEFORE using in ESO
    U_MAX = 0.6
    ux = np.clip(ux_raw, -U_MAX, U_MAX)
    uy = np.clip(uy_raw, -U_MAX, U_MAX)
    uz = np.clip(uz_raw, -U_MAX, U_MAX)

    # --- ESO updates ---
    eso_x.update(pos[0], ux)
    eso_y.update(pos[1], uy)
    eso_z.update(pos[2], uz)




    ux, uy, uz = np.clip([ux, uy, uz], -0.6, 0.6)

    # --- Convert to body-frame velocity command ---
    v_body_x, v_body_y = body_velocity_from_global(ux, uy, yaw)
    forward_desired = v_body_x
    sideways_desired = v_body_y
    yaw_desired = 0.0
    height_desired = z_ref

    cosy, siny = np.cos(yaw), np.sin(yaw)
    v_x_global = (pos[0] - prev_pos[0]) / dt
    v_y_global = (pos[1] - prev_pos[1]) / dt
    v_x =  v_x_global * cosy + v_y_global * siny
    v_y = -v_x_global * siny + v_y_global * cosy

    if np.isnan(ux) or np.isnan(uy) or np.isnan(uz):
        print(f"\n[NaN DETECTED] t={t:.2f}, dt={dt:.4f}")
        print(f"pos={pos}, target={target}")
        print(f"eso_x=({eso_x.x1:.2f},{eso_x.x2:.2f},{eso_x.d:.2f})")
        print(f"eso_y=({eso_y.x1:.2f},{eso_y.x2:.2f},{eso_y.d:.2f})")
        print(f"u_cmd=({ux},{uy},{uz})\n")


    # --- Inner PID velocity controller ---
    altitude = pos[2]
    motor_power = PID_CF.pid(
        dt,
        forward_desired, sideways_desired,
        yaw_desired, height_desired,
        roll, pitch, yaw_rate,
        altitude, v_x, v_y
    )

    # --- Apply motor speeds ---
    m1_motor, m2_motor, m3_motor, m4_motor = motors
    m1_motor.setVelocity(-motor_power[0])
    m2_motor.setVelocity( motor_power[1])
    m3_motor.setVelocity(-motor_power[2])
    m4_motor.setVelocity( motor_power[3])

    # --- Print every 0.5 s ---
    if int(t*10) % 5 == 0:
        print(f"t={t:4.1f} | Ref=({x_ref:+.2f},{y_ref:+.2f}) | Pos=({pos[0]:+.2f},{pos[1]:+.2f},{pos[2]:+.2f}) | ux={ux:+.2f}, uy={uy:+.2f}")
