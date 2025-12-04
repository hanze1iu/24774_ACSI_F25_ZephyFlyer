"""
Crazyflie outer-loop controller with 12-State ESO.
"""

from controller import Robot, Keyboard
import numpy as np
import sys
sys.path.append('../../../../controllers_shared/python_based')
from plotter import DataLogger
logger = DataLogger("eso_flight")

from pid_controller import pid_velocity_fixed_height_controller
from eso import AttitudeESO
from design_L import compute_L

# =========================
# ESO CONTROL MODE
# =========================
USE_ESO_FOR_CONTROL = True

# =========================
# Sim
# =========================
robot = Robot()
timestep = int(robot.getBasicTimeStep())
Ts = timestep / 1000.0

# =========================
# ESO initialization (12-state)
# =========================
m = 0.031
g = 9.81

# 1) Create ESO with zero gains first
eso = AttitudeESO(Ts, np.zeros((12,6)), m, g)

# 2) Design L
L = compute_L(eso, m)

# 3) Assign
eso.L = L

print(f"\n{'='*60}")
print(f"ESO MODE: {'ACTIVE' if USE_ESO_FOR_CONTROL else 'MONITORING ONLY'}")
print(f"{'='*60}\n")

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
# Wait for initialization
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

print(f"Sensors initialized: pos={pos0}, rpy={rpy0}")

# =========================
# Initialize ESO
# =========================
y_init = np.array([pos0[0], pos0[1], pos0[2], rpy0[0], rpy0[1], rpy0[2]])
eso.initialize_from_measurement(y_init)
print(f"ESO initialized")

# =========================
# Helper
# =========================
def body_velocity_from_global(vx_g, vy_g, yaw):
    cosy, siny = np.cos(yaw), np.sin(yaw)
    vx = vx_g * cosy + vy_g * siny
    vy = -vx_g * siny + vy_g * cosy
    return vx, vy

# =========================
# Controller
# =========================
PID_CF = pid_velocity_fixed_height_controller()
T_cmd = m * g
# target = np.array([0.0, 0.0, 0.5])

Kp_pos   = 0.6
Kd_pos   = 0.3
VEL_LIMIT = 0.8   # and clamp vx_des, vy_des to this

past_time = robot.getTime()
past_pos = pos0.copy()

PRINT_INTERVAL = 0.5
last_print_time = 0.0

# ===========================================================
# CIRCULAR TRAJECTORY GENERATOR (VELOCITY-BASED, LIKE REAL CODE)
# ===========================================================
class CircleTrajectory:
    def __init__(self, radius=0.5, z_height=0.5, period=10.0):
        self.radius = radius
        self.z = z_height
        self.T = period
        self.omega = 2 * np.pi / period
        
    def get(self, t):
        """Return desired position and velocity at time t.
        Circle passes THROUGH origin at t=0."""
        r = self.radius
        w = self.omega
        
        # Current angle (starts at -π so position begins at origin)
        angle = w * t - np.pi
        
        # Position - circle centered at (r, 0) so it passes through origin
        x = r + r * np.cos(angle)  # Center shifted right by r
        y = r * np.sin(angle)
        z = self.z
        
        # Velocity (tangent to circle)
        vx = -r * w * np.sin(angle)
        vy =  r * w * np.cos(angle)
        vz = 0.0
        
        return np.array([x, y, z]), np.array([vx, vy, vz])

trajectory = CircleTrajectory(
    radius=0.5,
    z_height=0.5,
    period=15.0
)

TAKEOFF_Z = 0.5
TAKEOFF_THRESHOLD = 0.45   # must reach this altitude before starting circle
CIRCLE_STARTED = False
CIRCLE_FINISHED = False

ESO_READY = False
ESO_INIT_TIME = 1.0     # seconds to wait before initializing ESO

imu_buffer = []
gps_buffer = []


# =========================
# MAIN LOOP
# =========================
print("Starting main control loop...")

while robot.step(timestep) != -1:
    t = robot.getTime()
    dt = max(t - past_time, 1e-6)
    past_time = t

    # Read sensors
    pos = np.array(gps.getValues())
    roll, pitch, yaw = imu.getRollPitchYaw()
    omega_body = np.array(gyro.getValues())
    yaw_rate = omega_body[2]

    if not (np.isfinite(pos).all() and np.isfinite([roll, pitch, yaw]).all()):
        print(f"Warning: Invalid sensor readings at t={t:.2f}")
        continue

    gps_velocity = (pos - past_pos) / dt
    past_pos = pos.copy()

    # ---------------------------------------------
    # ESO Initialization Delay
    # ---------------------------------------------
    if not ESO_READY:
        if t < ESO_INIT_TIME:
            imu_buffer.append([roll, pitch, yaw])
            gps_buffer.append(pos.tolist())
            continue  # skip control until ESO warmed up
        else:
            # compute stable averages
            avg_pos = np.mean(np.array(gps_buffer), axis=0)
            avg_rpy = np.mean(np.array(imu_buffer), axis=0)

            y_init = np.array([
                avg_pos[0], avg_pos[1], avg_pos[2],
                avg_rpy[0], avg_rpy[1], avg_rpy[2]
            ])

            eso.initialize_from_measurement(y_init)
            ESO_READY = True
            print(">>> ESO INITIALIZED AFTER DELAY. avg_pos=", avg_pos, " avg_rpy=", avg_rpy)

    # ---------------------------------------------
    # ESO UPDATE (ready or fallback, but always fill eso_* )
    # ---------------------------------------------
    if ESO_READY:
        y_meas = np.array([pos[0], pos[1], pos[2], roll, pitch, yaw])
        z_hat = eso.step(y_meas, T_cmd, omega_body)
    else:
        # fallback to IMU/GPS until ESO ready
        z_hat = np.zeros(12)
        z_hat[0:3] = pos
        z_hat[3:6] = gps_velocity
        z_hat[6:9] = [roll, pitch, yaw]

    # Extract states (common interface)
    eso_p = z_hat[0:3]
    eso_v = z_hat[3:6]
    eso_att = z_hat[6:9]
    eso_d_f = z_hat[9:12]
    eso_roll, eso_pitch, eso_yaw = eso_att

    # ---------------------------------------------
    # Choose control source (now ALWAYS defines ctrl_pos, etc.)
    # ---------------------------------------------
    if USE_ESO_FOR_CONTROL:
        ctrl_pos = eso_p
        ctrl_vx_world, ctrl_vy_world, ctrl_vz = eso_v
        ctrl_roll = eso_roll
        ctrl_pitch = eso_pitch
        ctrl_yaw = eso_yaw
        source_label = "ESO" if ESO_READY else "ESO (fallback from GPS)"
    else:
        ctrl_pos = pos
        ctrl_vx_world, ctrl_vy_world, ctrl_vz = gps_velocity
        ctrl_roll = roll
        ctrl_pitch = pitch
        ctrl_yaw = yaw
        source_label = "GPS+IMU"

    # ===========================================================
    # TRAJECTORY MANAGEMENT (LASSO PATTERN)
    # ===========================================================

    TAKEOFF_TIME = 3.0
    TAKEOFF_Z = 0.5
    ORIGIN = np.array([0.0, 0.0])
    CIRCLE_CENTER = np.array([0.5, 0.0])  # Circle center offset from origin

    if not CIRCLE_STARTED:
        # Smooth takeoff ramp at origin
        elapsed_takeoff = t - ESO_INIT_TIME
        z_cmd = TAKEOFF_Z * np.clip(elapsed_takeoff / TAKEOFF_TIME, 0.0, 1.0)
        
        target = np.array([0.0, 0.0, z_cmd])
        vel_ff = np.zeros(3)
        
        # Check if fully reached takeoff point
        if elapsed_takeoff >= TAKEOFF_TIME and abs(ctrl_pos[2] - TAKEOFF_Z) < 0.05:
            print(f">>> TAKEOFF COMPLETE at t={t:.2f}s — STARTING CIRCLE")
            CIRCLE_STARTED = True
            circle_start_time = t

    # ------------------ CIRCLE FLIGHT ----------------------
    elif CIRCLE_STARTED and not CIRCLE_FINISHED:
        rel_time = t - circle_start_time
        
        if rel_time <= trajectory.T:
            target, vel_ff = trajectory.get(rel_time)
        else:
            print(f">>> CIRCLE COMPLETE at t={t:.2f}s — RETURNING TO ORIGIN FOR LANDING")
            CIRCLE_FINISHED = True
            landing_start_time = t

    # ------------------ LANDING SEQUENCE -------------------
    elif CIRCLE_FINISHED:
        # First return horizontally to origin
        xy_err = ORIGIN - ctrl_pos[:2]
        xy_dist = np.linalg.norm(xy_err)
        
        if xy_dist > 0.05:
            # Move back to origin (maintain current altitude)
            target = np.array([ORIGIN[0], ORIGIN[1], ctrl_pos[2]])
            vel_ff = np.zeros(3)
        
        else:
            # Once at origin → descend smoothly
            descend_rate = -0.10   # m/s
            final_z = 0.0
            
            z_cmd = max(final_z, ctrl_pos[2] + descend_rate * dt)
            target = np.array([0.0, 0.0, z_cmd])
            vel_ff = np.zeros(3)
            
            if ctrl_pos[2] <= 0.10:  # Within 10cm of ground
                print(">>> LANDED — SHUTTING DOWN MOTORS")
                break



    # Position control with feedforward
    pos_err = target - ctrl_pos

    vx_des = Kp_pos * pos_err[0] + Kd_pos * (-ctrl_vx_world) + vel_ff[0]
    vy_des = Kp_pos * pos_err[1] + Kd_pos * (-ctrl_vy_world) + vel_ff[1]
    vz_des = Kp_pos * pos_err[2] + Kd_pos * (-ctrl_vz)       + vel_ff[2]

    # === Clamp lateral velocity commands ===
    vx_des = np.clip(vx_des, -VEL_LIMIT, VEL_LIMIT)
    vy_des = np.clip(vy_des, -VEL_LIMIT, VEL_LIMIT)

    # Transform to body frame
    v_body_x_des, v_body_y_des = body_velocity_from_global(vx_des, vy_des, ctrl_yaw)
    ctrl_vx_body, ctrl_vy_body = body_velocity_from_global(ctrl_vx_world, ctrl_vy_world, ctrl_yaw)

    # Inner PID - USE ESO ATTITUDE if enabled!
    motor_power = PID_CF.pid(
        dt,
        v_body_x_des,
        v_body_y_des,
        0.0,
        target[2],
        ctrl_roll,       # ESO or IMU
        ctrl_pitch,      # ESO or IMU
        yaw_rate,
        ctrl_pos[2],
        ctrl_vx_body,
        ctrl_vy_body
    )

    # Safety
    motor_power = np.clip(motor_power, 0, 600)

    if np.any(np.isnan(motor_power)) or np.any(np.isinf(motor_power)):
        print(f"ERROR: Invalid motor commands at t={t:.2f}")
        motor_power = np.array([100, 100, 100, 100])

    if abs(ctrl_roll) > np.pi/4 or abs(ctrl_pitch) > np.pi/4:
        print(f"WARNING: Extreme attitude at t={t:.2f}")
        motor_power = np.array([20, 20, 20, 20])

    # Apply motors
    motors[0].setVelocity(-motor_power[0])
    motors[1].setVelocity( motor_power[1])
    motors[2].setVelocity(-motor_power[2])
    motors[3].setVelocity( motor_power[3])

    # Log data
    logger.log(
        t,
        pos,
        eso_p, eso_v, eso_att,
        [roll, pitch, yaw],
        target,
        motor_power
    )
    # Prints
    if (t - last_print_time) >= PRINT_INTERVAL:
        last_print_time = t
        
        print(f"\n{'='*60}")
        print(f"Time: {t:.2f}s | Control Source: {source_label}")
        print(f"Position: GPS=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}), ESO=({eso_p[0]:+.3f},{eso_p[1]:+.3f},{eso_p[2]:+.3f})")
        print(f"Velocity: GPS=({gps_velocity[0]:+.3f},{gps_velocity[1]:+.3f},{gps_velocity[2]:+.3f}), ESO=({eso_v[0]:+.3f},{eso_v[1]:+.3f},{eso_v[2]:+.3f})")
        print(f"Attitude: IMU=({roll*180/np.pi:+.1f}°,{pitch*180/np.pi:+.1f}°,{yaw*180/np.pi:+.1f}°), ESO=({eso_roll*180/np.pi:+.1f}°,{eso_pitch*180/np.pi:+.1f}°,{eso_yaw*180/np.pi:+.1f}°)")
        print(f"Force Dist: ({eso_d_f[0]:+.4f},{eso_d_f[1]:+.4f},{eso_d_f[2]:+.4f})N")
        print(f"Motors: [{motor_power[0]:.1f},{motor_power[1]:.1f},{motor_power[2]:.1f},{motor_power[3]:.1f}]")

# When loop ends (simulation reset or closed), close the log
logger.close()
print("Log file closed.")