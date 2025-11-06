# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a robust control project based on the CrazyFlie 2.1+ drone platform. The codebase contains test scripts for drone control and hardware deck validation using the CrazyFlie Python library (cflib).

## Environment Setup

The project uses a Python virtual environment with dependencies managed through requirements.txt.

**Setup environment:**
```bash
source setup_env.sh
```

This script creates a `.venv` virtual environment, activates it, and installs all dependencies from requirements.txt.

**Manual activation (after initial setup):**
```bash
source .venv/bin/activate
```

**Install/update dependencies:**
```bash
pip install -r requirements.txt
```

## Running Scripts

All test scripts are located in `tester_files/` and should be run with the virtual environment activated.

**Basic connection test:**
```bash
python tester_files/connect_cf.py
```

**Flight demonstration (scripted path):**
```bash
python tester_files/test_flowdeck.py
```

**Multi-ranger deck test (obstacle avoidance):**
```bash
python tester_files/flight_demo.py
```

**Custom URI (optional):**
```bash
python tester_files/flight_demo.py radio://0/80/2M
```

## Architecture

### CrazyFlie Connection Pattern

All scripts follow a consistent pattern for connecting to the drone:

1. **Initialize CRTP drivers** with `cflib.crtp.init_drivers()`
2. **Scan for available interfaces** (or use predefined URI)
3. **Create SyncCrazyflie context** with the target URI
4. **Arm the drone** with `scf.cf.platform.send_arming_request(True)`
5. **Execute flight commands** within the context manager
6. **Automatic cleanup** on context exit

### Key Components

- **SyncCrazyflie**: Synchronous context manager for drone connection
- **MotionCommander**: High-level flight command interface for scripted movements
- **Multiranger**: Interface to multi-ranger deck for distance measurements in all directions
- **Cache**: Drone configuration cache stored in `./cache` directory

### URI Configuration

Default URI is `radio://0/80/2M`. This should be changed to match your specific radio channel configuration. URI can be:
- Hardcoded in the script's `URI` variable
- Passed as command-line argument (supported in flight_demo.py and test_rangerdeck.py)
- Auto-detected via `cflib.crtp.scan_interfaces()`

### Hardware Requirements

The test scripts assume the following hardware setup:
- CrazyFlie 2.1+ drone
- Crazyradio PA USB dongle
- Flow deck (for test_flowdeck.py)
- Multi-ranger deck (for flight_demo.py and test_rangerdeck.py)

## File Organization

### Hardware Test Scripts (tester_files/)
- `tester_files/connect_cf.py`: Basic connection and interface scanning
- `tester_files/test_flowdeck.py`: Scripted flight path demonstration using MotionCommander
- `tester_files/flight_demo.py`: Interactive obstacle avoidance using multi-ranger deck
- `tester_files/test_rangerdeck.py`: Duplicate of flight_demo.py for ranger deck testing

Note: `flight_demo.py` and `test_rangerdeck.py` are identical implementations.

### Simulation Scripts (simulation/)

#### MuJoCo Environment
The project includes a MuJoCo-based physics simulation for CrazyFlie 2.1 located in `simulation/crazyflie_sim/`:
- `scene.xml`: Main simulation scene with ground plane and lighting
- `cf2.xml`: CrazyFlie 2.1 drone model with actuators and sensors
- `assets/`: 3D mesh files for visual and collision geometry

The MuJoCo model uses a simplified control abstraction:
- **4 actuators**: body_thrust, x_moment (roll), y_moment (pitch), z_moment (yaw)
- **3 sensors**: gyroscope, accelerometer, orientation (quaternion)
- **Hover keyframe**: Predefined hover state at 0.1m with thrust=0.26487N

**Important**: This model uses direct force/moment control rather than individual motor RPM. Propellers in the visualization do not rotate - this is intentional for the simplified dynamics model.

#### Simulation Scripts

**Basic hover demonstration:**
```bash
python simulation/hover_demo.py
```
Demonstrates constant-thrust hover control using the predefined hover keyframe. The drone stabilizes at approximately 0.1m altitude.

**Keyboard-controlled flight:**
```bash
python simulation/keyboard_control.py
```
Interactive manual control with keyboard inputs:
- Arrow keys: Pitch (↑/↓) and Roll (←/→)
- W/S: Altitude control (increase/decrease thrust)
- A/D: Yaw rotation (left/right)
- Space: Reset to hover state
- ESC: Exit simulation

Controls are incremental with automatic damping for smooth flight behavior.

**Cascade PID controllers:**
```bash
python simulation/cascade_pid_control.py      # Basic cascade PID
python simulation/cascade_pid_improved.py     # Enhanced stability version
python simulation/cascade_pid_tuned.py        # Optimized for faster convergence
```
Three versions of cascade PID controller with progressive improvements:
- **Basic**: Two-layer control with motor mixing and anti-windup
- **Improved**: Additional stability features (conditional integration, tilt limiting, derivative filtering)
- **Tuned**: Relaxed integration conditions (3.0m threshold) and optimized gains for faster response

**Viewer-only mode (no control):**
```bash
python -m mujoco.viewer --mjcf simulation/crazyflie_sim/scene.xml
```
Launch MuJoCo's built-in viewer to inspect the model without running control scripts.

#### Simulation Development Notes

- All simulation scripts require `mujoco>=3.0.0` (see requirements.txt)
- The keyboard controller uses increased gains for better visibility (moment_increment=0.001)
- Debug output includes position, velocity, and all control deltas
- The model is cross-platform: works on Windows, Linux, macOS
- WSL2 users need X Server (VcXsrv) for graphics; native Windows recommended for best experience

### Webots Environment (Alternative Simulation)

The project now includes access to the official Bitcraze Webots simulation through the `crazyflie-simulation` repository (cloned locally, not tracked in git).

**Location**: `/crazyflie-simulation/` (gitignored external repository)

#### Key Files

**World files**: `crazyflie-simulation/simulator_files/webots/worlds/`

Basic Demos:
- `crazyflie_world.wbt` - Basic keyboard control in open environment
- `crazyflie_apartement.wbt` - Indoor environment with wall following

Advanced Control Demos:
- `crazyflie_simple_pid.wbt` - Simple PID controller demonstration
- `crazyflie_simple_pid_fixed.wbt` - Fixed/improved version of simple PID
- `crazyflie_trajectory.wbt` - **Enhanced trajectory controller with XY position outer loop (PD+I)**
  - Features: Low-pass velocity filtering, integral for wind rejection, anti-windup
  - Parameters: Kp_xy=0.6, Kd_xy=0.3, Ki_xy=0.05, v_xy_max=0.3 m/s, tau_v=0.1s
  - Disturbance testing: Impulse forces and continuous wind simulation
- `crazyflie_disturbance.wbt` - Disturbance rejection testing environment

**Controllers**: `crazyflie-simulation/simulator_files/webots/controllers/`
- `crazyflie_controller_py/` - Python keyboard controller
- `crazyflie_controller_py_firmware_pid/` - Firmware-based PID
- `crazyflie_controller_py_wallfollowing/` - Wall following demo
- `crazyflie_simple_pid/` - Simple PID implementation
- `crazyflie_trajectory/` - **Enhanced trajectory controller with XY outer loop (PD+I)**
  - Outer loop: Position error → Velocity command (with integral for constant wind)
  - Inner loop: Official Bitcraze velocity controller (unchanged)
  - Press 'P' to view outer loop status (integrals, filtered velocity)

**Shared PID controller**: `crazyflie-simulation/controllers_shared/python_based/pid_controller.py`

#### Installation and Setup

**See [WEBOTS_QUICKSTART.md](WEBOTS_QUICKSTART.md) for 5-minute setup guide.**

**Installation requirements**:
1. Webots R2023b or newer (Windows recommended for WSL2 users)
2. Download from https://cyberbotics.com/
3. No additional Python dependencies needed for basic keyboard control
4. cflib already installed in project venv for advanced features

**Running Webots**:
```bash
# On Windows, open Webots GUI and load:
# File → Open World... → Navigate to:
\\wsl.localhost\Ubuntu\home\hanzel\24774_project\24774_ACSI_F25_ZephyFlyer\crazyflie-simulation\simulator_files\webots\worlds\crazyflie_world.wbt
```

**Keyboard controls in Webots**:
- Arrow keys: Forward/backward/left/right
- W/S: Up/down (altitude)
- Q/E: Yaw left/right
- A: Toggle wall following (in apartment world)

#### Webots vs MuJoCo

**Use Webots for**:
- Quick prototyping and demos
- Visual debugging in complex environments
- Testing with official Bitcraze parameters
- Reference implementation comparison

**Use MuJoCo for**:
- High-precision control algorithm development
- Parameter tuning and optimization
- Research-grade simulation accuracy
- Custom dynamics modeling

#### Important: Official PID Parameters

The Webots controller uses official Bitcraze PID gains that differ significantly from our MuJoCo implementation:

```python
# Webots (official Bitcraze parameters)
gains = {
    "kp_att_rp": 0.5,    # Roll/pitch attitude
    "kd_att_rp": 0.1,
    "kp_att_y": 1.0,     # Yaw attitude
    "kd_att_y": 0.5,
    "kp_vel_xy": 2.0,    # XY velocity
    "kd_vel_xy": 0.5,
    "kp_z": 10.0,        # Altitude
    "ki_z": 5.0,
    "kd_z": 5.0
}
```

**Key differences from MuJoCo implementation**:
- Attitude control gains are **25x higher** (0.5 vs 0.02)
- This may explain MuJoCo PID instability issues
- Consider adjusting MuJoCo inner loop gains based on these values

#### Documentation

- **[PROJECT_STATUS.md](PROJECT_STATUS.md)** - Current development status and technical analysis
- **[WEBOTS_SETUP.md](WEBOTS_SETUP.md)** - Complete Webots installation guide
- **[WEBOTS_QUICKSTART.md](WEBOTS_QUICKSTART.md)** - Quick start guide
