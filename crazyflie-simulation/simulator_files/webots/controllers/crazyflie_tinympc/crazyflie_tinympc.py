"""
TinyMPC Hover Controller for CrazyFlie in Webots

This controller uses TinyMPC to stabilize the CrazyFlie at a fixed hover position.
The MPC formulation uses a linearized quadrotor model discretized at 100Hz.

State vector (12 states):
    x = [px, py, pz, vx, vy, vz, phi, theta, psi, p, q, r]
    - px, py, pz: position in world frame (m)
    - vx, vy, vz: velocity in world frame (m/s)
    - phi, theta, psi: roll, pitch, yaw angles (rad)
    - p, q, r: angular rates (rad/s)

Control input (4 inputs):
    u = [thrust_delta, roll_moment, pitch_moment, yaw_moment]
    - thrust_delta: deviation from hover thrust
    - roll/pitch/yaw moments: body torques

Author: Claude Code
Date: 2025-11-23
"""

import numpy as np
import tinympc
from controller import Robot, Motor, InertialUnit, GPS, Gyro, Keyboard
from math import cos, sin, pi


def get_linearized_model(dt=0.01, mass=0.027, g=9.81,
                         Ixx=1.6e-5, Iyy=1.6e-5, Izz=2.9e-5):
    """
    Get linearized discrete-time state-space model for quadrotor near hover.

    State: x = [px, py, pz, vx, vy, vz, phi, theta, psi, p, q, r]
    Input: u = [thrust_delta, roll_moment, pitch_moment, yaw_moment]

    Continuous dynamics linearized at hover:
        ṗx = vx
        ṗy = vy
        ṗz = vz
        v̇x = g * theta  (small angle approx)
        v̇y = -g * phi
        v̇z = thrust_delta / mass
        φ̇ = p
        θ̇ = q
        ψ̇ = r
        ṗ = roll_moment / Ixx
        q̇ = pitch_moment / Iyy
        ṙ = yaw_moment / Izz

    Returns:
        A: 12x12 discrete state transition matrix
        B: 12x4 discrete input matrix
    """
    nx = 12  # number of states
    nu = 4   # number of inputs

    # Continuous-time A matrix
    A_cont = np.zeros((nx, nx))

    # Position dynamics: ṗ = v
    A_cont[0, 3] = 1.0  # ṗx = vx
    A_cont[1, 4] = 1.0  # ṗy = vy
    A_cont[2, 5] = 1.0  # ṗz = vz

    # Velocity dynamics (linearized): v̇ = g * angles
    A_cont[3, 7] = g     # v̇x = g * theta
    A_cont[4, 6] = -g    # v̇y = -g * phi

    # Attitude dynamics: θ̇ = ω
    A_cont[6, 9] = 1.0   # φ̇ = p
    A_cont[7, 10] = 1.0  # θ̇ = q
    A_cont[8, 11] = 1.0  # ψ̇ = r

    # Continuous-time B matrix
    B_cont = np.zeros((nx, nu))
    B_cont[5, 0] = 1.0 / mass      # v̇z from thrust
    B_cont[9, 1] = 1.0 / Ixx       # ṗ from roll moment
    B_cont[10, 2] = 1.0 / Iyy      # q̇ from pitch moment
    B_cont[11, 3] = 1.0 / Izz      # ṙ from yaw moment

    # Discretization using Euler method: A_d = I + A*dt, B_d = B*dt
    A_disc = np.eye(nx) + A_cont * dt
    B_disc = B_cont * dt

    # TinyMPC requires Fortran-contiguous arrays
    A_disc = np.asfortranarray(A_disc, dtype=np.float64)
    B_disc = np.asfortranarray(B_disc, dtype=np.float64)

    return A_disc, B_disc


def get_cost_matrices(nx=12, nu=4):
    """
    Get Q and R cost matrices for MPC.

    Q: state cost (penalizes deviation from reference)
    R: input cost (penalizes control effort)

    Returns:
        Q: 12x12 state cost matrix
        R: 4x4 input cost matrix
    """
    # State cost: prioritize position tracking, then velocity, then attitude
    Q = np.diag([
        10.0, 10.0, 20.0,    # position (x, y, z) - z higher for altitude hold
        1.0, 1.0, 1.0,       # velocity (vx, vy, vz)
        5.0, 5.0, 2.0,       # angles (phi, theta, psi) - penalize roll/pitch more
        0.1, 0.1, 0.1        # angular rates (p, q, r)
    ])

    # Input cost: penalize aggressive control
    R = np.diag([
        1.0,     # thrust
        10.0,    # roll moment
        10.0,    # pitch moment
        10.0     # yaw moment
    ])

    # TinyMPC requires Fortran-contiguous arrays
    Q = np.asfortranarray(Q, dtype=np.float64)
    R = np.asfortranarray(R, dtype=np.float64)

    return Q, R


class TinyMPCController:
    """TinyMPC-based controller for CrazyFlie hover."""

    def __init__(self, dt=0.01, hover_height=0.5, horizon=10):
        """
        Initialize TinyMPC controller.

        Args:
            dt: control timestep (s), default 0.01 (100Hz)
            hover_height: target hover altitude (m)
            horizon: MPC prediction horizon (N)
        """
        self.dt = dt
        self.hover_height = hover_height
        self.horizon = horizon

        # CrazyFlie parameters (approximate values for Webots model)
        self.mass = 0.027  # kg
        self.g = 9.81      # m/s^2
        self.Ixx = 1.6e-5  # kg*m^2
        self.Iyy = 1.6e-5
        self.Izz = 2.9e-5

        # Get linearized model
        self.A, self.B = get_linearized_model(
            dt=dt, mass=self.mass, g=self.g,
            Ixx=self.Ixx, Iyy=self.Iyy, Izz=self.Izz
        )

        # Get cost matrices
        self.Q, self.R = get_cost_matrices()

        # State and input dimensions
        self.nx = 12
        self.nu = 4

        # Initialize TinyMPC solver (without constraints for initial testing)
        self.solver = tinympc.TinyMPC()
        self.solver.setup(self.A, self.B, self.Q, self.R, self.horizon)

        # Reference state (hover position)
        self.x_ref = np.zeros(self.nx, dtype=np.float64)
        self.x_ref[2] = hover_height  # z position

        # Reference input (zero for hover - thrust compensated in motor mixing)
        self.u_ref = np.zeros(self.nu, dtype=np.float64)

        # Set references
        self.solver.set_x_ref(self.x_ref)
        self.solver.set_u_ref(self.u_ref)

        # For motor mixing
        self.hover_thrust_cmd = 48.0  # baseline motor command for hover

        print(f"TinyMPC Controller initialized:")
        print(f"  - Timestep: {dt*1000:.0f}ms ({1/dt:.0f}Hz)")
        print(f"  - Horizon: {horizon} steps ({horizon*dt*1000:.0f}ms)")
        print(f"  - Hover height: {hover_height}m")
        print(f"  - State dim: {self.nx}, Input dim: {self.nu}")

    def set_target_position(self, x, y, z):
        """Update target hover position."""
        self.x_ref[0] = float(x)
        self.x_ref[1] = float(y)
        self.x_ref[2] = float(z)
        self.solver.set_x_ref(self.x_ref)

    def get_state(self, gps_values, imu_values, gyro_values,
                  past_pos, past_time, current_time):
        """
        Construct state vector from sensor readings.

        Args:
            gps_values: [x, y, z] position
            imu_values: [roll, pitch, yaw] angles
            gyro_values: [gx, gy, gz] angular rates
            past_pos: [x, y, z] position at previous timestep
            past_time: previous timestamp
            current_time: current timestamp

        Returns:
            x: 12-element state vector
        """
        dt = current_time - past_time
        if dt < 1e-6:
            dt = self.dt

        # Position
        px, py, pz = gps_values

        # Velocity (numerical differentiation)
        vx = (px - past_pos[0]) / dt
        vy = (py - past_pos[1]) / dt
        vz = (pz - past_pos[2]) / dt

        # Attitude
        phi, theta, psi = imu_values  # roll, pitch, yaw

        # Angular rates
        p, q, r = gyro_values

        return np.array([px, py, pz, vx, vy, vz, phi, theta, psi, p, q, r], dtype=np.float64)

    def compute_control(self, state):
        """
        Compute optimal control using TinyMPC.

        Args:
            state: 12-element state vector

        Returns:
            u: 4-element control vector [thrust_delta, roll_mom, pitch_mom, yaw_mom]
        """
        # Set current state
        self.solver.set_x0(state)

        # Solve MPC problem
        solution = self.solver.solve()

        # Get first control input
        u = solution['controls'][:, 0] if solution['controls'].ndim > 1 else solution['controls']

        return u

    def control_to_motors(self, u):
        """
        Convert MPC control output to motor commands.

        Args:
            u: [thrust_delta, roll_moment, pitch_moment, yaw_moment]

        Returns:
            motors: [m1, m2, m3, m4] motor velocities
        """
        thrust_delta, roll_mom, pitch_mom, yaw_mom = u

        # Scale factors to convert MPC output to motor command space
        thrust_scale = 100.0   # thrust delta to motor units
        moment_scale = 5000.0  # moment to motor units

        # Base thrust command (hover)
        alt_cmd = self.hover_thrust_cmd + thrust_delta * thrust_scale

        # Moment commands
        roll_cmd = roll_mom * moment_scale
        pitch_cmd = pitch_mom * moment_scale
        yaw_cmd = yaw_mom * moment_scale

        # Motor mixing (same as PID controller)
        m1 = alt_cmd - roll_cmd + pitch_cmd + yaw_cmd
        m2 = alt_cmd - roll_cmd - pitch_cmd - yaw_cmd
        m3 = alt_cmd + roll_cmd - pitch_cmd + yaw_cmd
        m4 = alt_cmd + roll_cmd + pitch_cmd - yaw_cmd

        # Clamp to valid range
        m1 = np.clip(m1, 0, 600)
        m2 = np.clip(m2, 0, 600)
        m3 = np.clip(m3, 0, 600)
        m4 = np.clip(m4, 0, 600)

        return [m1, m2, m3, m4]


def main():
    """Main control loop."""
    # Initialize Webots robot
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # For 100Hz control, we need timestep <= 10ms
    # Webots default is often 32ms, so we'll run MPC at simulation rate
    control_dt = timestep / 1000.0  # convert to seconds

    print(f"\nWebots timestep: {timestep}ms")

    # Initialize motors
    m1_motor = robot.getDevice("m1_motor")
    m1_motor.setPosition(float('inf'))
    m1_motor.setVelocity(-1)

    m2_motor = robot.getDevice("m2_motor")
    m2_motor.setPosition(float('inf'))
    m2_motor.setVelocity(1)

    m3_motor = robot.getDevice("m3_motor")
    m3_motor.setPosition(float('inf'))
    m3_motor.setVelocity(-1)

    m4_motor = robot.getDevice("m4_motor")
    m4_motor.setPosition(float('inf'))
    m4_motor.setVelocity(1)

    # Initialize sensors
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)

    gps = robot.getDevice("gps")
    gps.enable(timestep)

    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)

    # Initialize keyboard
    keyboard = Keyboard()
    keyboard.enable(timestep)

    # Initialize TinyMPC controller
    hover_height = 0.5  # Target hover height
    horizon = 10        # MPC horizon (N)

    mpc = TinyMPCController(
        dt=control_dt,
        hover_height=hover_height,
        horizon=horizon
    )

    # Target position
    target_x = 0.0
    target_y = 0.0
    target_z = hover_height

    # State tracking
    past_pos = [0.0, 0.0, 0.0]
    past_time = 0.0

    # Logging
    last_print_time = 0.0
    print_interval = 1.0  # Print every 1 second

    print("\n" + "="*60)
    print("TinyMPC Hover Controller for CrazyFlie")
    print("="*60)
    print(f"\nTarget: hover at ({target_x}, {target_y}, {target_z})m")
    print("\nControls:")
    print("  Arrow keys: Move target X/Y")
    print("  W/S: Adjust target altitude")
    print("  R: Reset to origin")
    print("  SPACE: Print status")
    print("="*60 + "\n")

    # Wait for sensors to initialize
    for _ in range(5):
        robot.step(timestep)

    # Main loop
    while robot.step(timestep) != -1:
        current_time = robot.getTime()

        # Get sensor readings
        gps_values = gps.getValues()
        imu_values = imu.getRollPitchYaw()
        gyro_values = gyro.getValues()

        # Handle keyboard input
        key = keyboard.getKey()
        while key > 0:
            if key == Keyboard.UP:
                target_x += 0.1
                mpc.set_target_position(target_x, target_y, target_z)
                print(f"Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == Keyboard.DOWN:
                target_x -= 0.1
                mpc.set_target_position(target_x, target_y, target_z)
                print(f"Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == Keyboard.LEFT:
                target_y += 0.1
                mpc.set_target_position(target_x, target_y, target_z)
                print(f"Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == Keyboard.RIGHT:
                target_y -= 0.1
                mpc.set_target_position(target_x, target_y, target_z)
                print(f"Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == ord('W'):
                target_z += 0.1
                mpc.set_target_position(target_x, target_y, target_z)
                print(f"Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == ord('S'):
                target_z = max(0.1, target_z - 0.1)
                mpc.set_target_position(target_x, target_y, target_z)
                print(f"Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == ord('R') or key == ord('r'):
                target_x, target_y, target_z = 0.0, 0.0, hover_height
                mpc.set_target_position(target_x, target_y, target_z)
                print(f"Reset target to ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == ord(' '):
                pos_err = np.sqrt((gps_values[0]-target_x)**2 +
                                  (gps_values[1]-target_y)**2 +
                                  (gps_values[2]-target_z)**2)
                print(f"\nStatus at t={current_time:.2f}s:")
                print(f"  Position: ({gps_values[0]:.3f}, {gps_values[1]:.3f}, {gps_values[2]:.3f})")
                print(f"  Target:   ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
                print(f"  Error:    {pos_err:.4f}m")
                print(f"  Attitude: roll={imu_values[0]*180/pi:.1f}, pitch={imu_values[1]*180/pi:.1f}, yaw={imu_values[2]*180/pi:.1f} deg\n")
            key = keyboard.getKey()

        # Construct state vector
        state = mpc.get_state(
            gps_values, imu_values, gyro_values,
            past_pos, past_time, current_time
        )

        # Compute MPC control
        u = mpc.compute_control(state)

        # Convert to motor commands
        motors = mpc.control_to_motors(u)

        # Apply motor commands
        m1_motor.setVelocity(-motors[0])
        m2_motor.setVelocity(motors[1])
        m3_motor.setVelocity(-motors[2])
        m4_motor.setVelocity(motors[3])

        # Periodic status print
        if current_time - last_print_time >= print_interval:
            pos_err = np.sqrt((gps_values[0]-target_x)**2 +
                              (gps_values[1]-target_y)**2 +
                              (gps_values[2]-target_z)**2)
            print(f"[{current_time:6.1f}s] Pos:({gps_values[0]:+.3f},{gps_values[1]:+.3f},{gps_values[2]:+.3f}) | "
                  f"Err:{pos_err:.4f}m | u:[{u[0]:.4f},{u[1]:.6f},{u[2]:.6f},{u[3]:.6f}]")
            last_print_time = current_time

        # Update past state
        past_pos = list(gps_values)
        past_time = current_time


if __name__ == '__main__':
    main()
