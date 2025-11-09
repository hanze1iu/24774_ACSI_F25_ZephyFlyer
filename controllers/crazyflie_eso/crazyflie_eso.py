# crazyflie_eso.py
from controller import Robot, Keyboard
import numpy as np
from eso import ESO

# ========================
# Simulation setup
# ========================
robot = Robot()
timestep = int(robot.getBasicTimeStep())  # ms
Ts = timestep / 1000.0

keyboard = Keyboard()
keyboard.enable(timestep)

# === Sensors - Safe device retrieval ===
def get_device_safe(robot, name):
    """Safely get a device, return None if not found"""
    device = robot.getDevice(name)
    if device is None:
        print(f"Warning: Device '{name}' not found, skipping...")
    return device

imu = get_device_safe(robot, 'inertial_unit')
gyro = get_device_safe(robot, 'gyro')
gps = get_device_safe(robot, 'gps')

# Enable only available sensors
available_sensors = []
if imu is not None:
    imu.enable(timestep)
    available_sensors.append('imu')
if gyro is not None:
    gyro.enable(timestep)
    available_sensors.append('gyro')
if gps is not None:
    gps.enable(timestep)
    available_sensors.append('gps')

print(f"Available sensors: {available_sensors}")

# Check critical sensors
if gps is None:
    raise RuntimeError("GPS sensor is required but not found!")
if imu is None:
    raise RuntimeError("IMU sensor is required but not found!")

# Initialize orientation
roll, pitch, yaw = imu.getRollPitchYaw()

# === Motors ===
motors = [robot.getDevice(f"m{i+1}_motor") for i in range(4)]
# Check if that naming didn't work, try alternative names
if motors[0] is None:
    motors = [get_device_safe(robot, f"motor{i+1}") for i in range(4)]
if motors[0] is None:
    # Try another common naming convention
    motors = [get_device_safe(robot, f"m{i}") for i in range(1, 5)]

for m in motors:
    if m is not None:
        m.setPosition(float('inf'))
        m.setVelocity(0.0)

# ========================
# ADRC parameters
# ========================
b_hat = 1.0
w_c = 6.0
w_o = 8.0 * w_c
Kp = w_c ** 2
Kd = 2.0 * w_c

# ESO per axis
eso_x = ESO(Ts, b_hat, w_o)
eso_y = ESO(Ts, b_hat, w_o)
eso_z = ESO(Ts, b_hat, w_o)

target = np.array([0.0, 0.0, 0.5])  # desired position

m = 0.027  # mass (kg)
g = 9.81

# ========================
# Helper
# ========================
def mix_to_motors(thrust, tau_phi, tau_theta, tau_psi):
    L = 0.046
    k_tau = 1e-7
    A = np.array([
        [1,  1,  1,  1],
        [0,  L,  0, -L],
        [-L, 0,  L,  0],
        [ k_tau, -k_tau, k_tau, -k_tau]
    ])
    wrench = np.array([thrust, tau_phi, tau_theta, tau_psi])
    f = np.linalg.lstsq(A, wrench, rcond=None)[0]
    return np.clip(f, 0.0, 1.0)

# ========================
# Main control loop
# ========================
print_interval = int(0.2 / Ts)   # print every 0.2s
step_count = 0

while robot.step(timestep) != -1:
    step_count += 1

    # --- Measurements ---
    pos = np.array(gps.getValues())      # [x, y, z]
    roll, pitch, yaw = imu.getRollPitchYaw()

    # Optional: get gyro data if available
    if gyro is not None:
        gyro_rates = gyro.getValues()  # [wx, wy, wz] in rad/s

    # --- ESO Control per axis ---
    ux = (Kp * (target[0] - eso_x.x1) - Kd * eso_x.x2 - eso_x.d) / b_hat
    uy = (Kp * (target[1] - eso_y.x1) - Kd * eso_y.x2 - eso_y.d) / b_hat
    uz = (Kp * (target[2] - eso_z.x1) - Kd * eso_z.x2 - eso_z.d) / b_hat

    eso_x.update(pos[0], ux)
    eso_y.update(pos[1], uy)
    eso_z.update(pos[2], uz)

    # --- Convert to attitude/thrust commands ---
    phi_des = uy / g
    theta_des = -ux / g
    thrust = m * (g + uz)

    tau_phi   = 0.02 * (phi_des - roll)
    tau_theta = 0.02 * (theta_des - pitch)
    tau_psi   = 0.01 * (0.0 - yaw)

    motor_forces = mix_to_motors(thrust, tau_phi, tau_theta, tau_psi)
    for i, mtr in enumerate(motors):
        if mtr is not None:
            mtr.setVelocity(1000 * np.sqrt(max(motor_forces[i], 0.0)))

    # --- Print ESO internal states every 0.2 s ---
    if step_count % print_interval == 0:
        print(f"\n[Step {step_count}]")
        print(f"Pos: {pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}")
        print(f"Roll: {roll:+.3f}, Pitch: {pitch:+.3f}, Yaw: {yaw:+.3f}")
        print(f"ESO-X: x1={eso_x.x1:+.3f}, x2={eso_x.x2:+.3f}, d={eso_x.d:+.3f}, u={ux:+.3f}")
        print(f"ESO-Y: x1={eso_y.x1:+.3f}, x2={eso_y.x2:+.3f}, d={eso_y.d:+.3f}, u={uy:+.3f}")
        print(f"ESO-Z: x1={eso_z.x1:+.3f}, x2={eso_z.x2:+.3f}, d={eso_z.d:+.3f}, u={uz:+.3f}")
