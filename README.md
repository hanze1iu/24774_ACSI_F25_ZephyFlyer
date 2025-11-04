# 24774_ACSI_F25_ZephyFlyer
Repo for robust control project based on CrazyFlie 2.1+ drone and disturbance.

## 🧠 Simulation

This repository includes **two simulation environments** for the Crazyflie 2.1 drone:

1. **MuJoCo** - High-precision physics simulation for control algorithm development
2. **Webots** - Official Bitcraze simulation for rapid prototyping and testing

### 📊 Simulation Comparison

| Feature | MuJoCo | Webots |
|---------|--------|--------|
| **Physics Accuracy** | High | Medium |
| **Official Support** | General physics engine | CrazyFlie official |
| **Maintenance** | Active (DeepMind) | Limited (archived) |
| **Best For** | Control algorithm R&D | Quick demos, testing |
| **Documentation** | See below | [WEBOTS_SETUP.md](WEBOTS_SETUP.md) |

### 📖 Documentation

- **[PROJECT_STATUS.md](PROJECT_STATUS.md)** - Current development status, MuJoCo PID issues, technical analysis
- **[WEBOTS_QUICKSTART.md](WEBOTS_QUICKSTART.md)** - 5-minute Webots setup and testing guide
- **[WEBOTS_SETUP.md](WEBOTS_SETUP.md)** - Complete Webots installation and configuration

---

## MuJoCo Simulation

High-precision physics simulation for control algorithm development.

### Setup
```bash
# Activate virtual environment and install dependencies
source setup_env.sh
```

### Running Simulations

#### 1. Basic MuJoCo Viewer (No Control)
Launch the simulation viewer to inspect the model:
```bash
python -m mujoco.viewer --mjcf simulation/crazyflie_sim/scene.xml
```

#### 2. Hover Demonstration (Simple Hover Control)
Run the hover control demo to see the drone stabilize at 0.1m altitude:
```bash
python simulation/hover_demo.py
```
This script applies constant thrust based on the hover keyframe to keep the drone stable in the air

#### 3. Keyboard Control (Interactive Manual Control)
Fly the drone manually using keyboard controls:
```bash
python simulation/keyboard_control.py
```

**Keyboard Controls:**
- **Arrow Keys**: Pitch (↑/↓) and Roll (←/→)
- **W/S**: Increase/Decrease altitude (thrust)
- **A/D**: Yaw left/right (rotation)
- **Space**: Reset to hover position
- **ESC**: Exit simulation

The drone starts in hover mode, and you can add control inputs incrementally. Controls are damped for smooth flight.

#### 4. Cascade PID Control (Autonomous Position Control)
Run the cascade PID controller to autonomously fly to a target position:
```bash
python simulation/cascade_pid_control.py
```

**Features:**
- **Two-layer control**: Outer loop (position) + Inner loop (attitude)
- **Motor mixing**: Simulates 4 individual motor thrusts with saturation handling
- **Target position**: Default (0, 0, 0.5) meters - flies to 0.5m altitude
- **Real-time feedback**: Position error, motor thrusts, saturation warnings
- **Anti-windup**: PID integral protection to prevent overshoot

The controller uses a cascade architecture where the outer loop computes desired attitude from position error, and the inner loop tracks that attitude. Motor allocation distributes forces/moments to 4 individual motors with proper saturation handling.

#### 5. Improved Cascade PID (Enhanced Stability)
Run the improved version with better stability and anti-windup:
```bash
python simulation/cascade_pid_improved.py
```

**Improvements over basic cascade PID:**
- **Conditional integration**: Disables integral term when error > 0.5m to prevent windup
- **Integral clamping**: Explicit limits on integral accumulation
- **Reduced Ki gains**: Less aggressive integration (50-60% reduction)
- **Tilt angle limiting**: Maximum 15° tilt to prevent extreme maneuvers
- **Emergency reset**: Automatically resets PID on large errors (> 2m)
- **Derivative filtering**: Low-pass filter on D-term to reduce noise amplification
- **Enhanced diagnostics**: Detailed PID term logging and reset counter

Use this version if the basic PID becomes unstable after extended runtime.

---

## Webots Simulation

Official Bitcraze simulation environment for rapid prototyping and visualization.

### Quick Start (5 minutes)

**See [WEBOTS_QUICKSTART.md](WEBOTS_QUICKSTART.md) for detailed instructions.**

1. **Download Webots**: Visit https://cyberbotics.com/ and download Windows version
2. **Install**: Run installer with default options
3. **Open World**: `File` → `Open World...` → Navigate to:
   ```
   \\wsl.localhost\Ubuntu\home\hanzel\24774_project\24774_ACSI_F25_ZephyFlyer\crazyflie-simulation\simulator_files\webots\worlds\crazyflie_world.wbt
   ```
4. **Run**: Click ▶️ Play button
5. **Control**: Click 3D window, use arrow keys to fly

### Available Worlds

The `crazyflie-simulation/` repository (cloned locally, not tracked in git) contains:

- **`crazyflie_world.wbt`** - Basic keyboard control in open environment
- **`crazyflie_apartement.wbt`** - Indoor scene with wall-following capability

### Key Finding: PID Parameters

Webots uses **official Bitcraze PID parameters** that differ significantly from our MuJoCo implementation:

```python
# Webots (official)         # MuJoCo (ours)
kp_att_rp: 0.5               kp_att_rp: 0.02  # 25x difference!
kd_att_rp: 0.1               kd_att_rp: 0.004 # 25x difference!
```

**This parameter analysis may help resolve MuJoCo PID instability issues.**

### Documentation

- **[WEBOTS_QUICKSTART.md](WEBOTS_QUICKSTART.md)** - Quick start guide
- **[WEBOTS_SETUP.md](WEBOTS_SETUP.md)** - Complete setup, comparison, troubleshooting
- **Controller source**: `crazyflie-simulation/controllers_shared/python_based/pid_controller.py`

---

## Project Status

See **[PROJECT_STATUS.md](PROJECT_STATUS.md)** for:
- Current MuJoCo PID controller development status
- Known stability issues and debugging strategies
- Technical architecture and parameter documentation
- Comparison with Webots parameters
