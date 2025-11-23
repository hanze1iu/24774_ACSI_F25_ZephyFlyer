# TinyMPC Hover Controller for CrazyFlie

This controller uses TinyMPC (Model Predictive Control) to stabilize the CrazyFlie at a fixed hover position.

## Overview

### State-Space Model (12 states)

The controller uses a linearized quadrotor model:

```
State: x = [px, py, pz, vx, vy, vz, phi, theta, psi, p, q, r]

Where:
- px, py, pz: position in world frame (m)
- vx, vy, vz: velocity in world frame (m/s)
- phi, theta, psi: roll, pitch, yaw angles (rad)
- p, q, r: angular rates (rad/s)
```

### Control Input (4 inputs)

```
u = [thrust_delta, roll_moment, pitch_moment, yaw_moment]
```

### MPC Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Timestep | 10ms | 100Hz control rate |
| Horizon (N) | 10 | 100ms prediction window |
| Hover Height | 0.5m | Default target altitude |

### Cost Matrices

**State cost Q** (diagonal):
- Position (x,y,z): [10, 10, 20]
- Velocity: [1, 1, 1]
- Angles: [5, 5, 2]
- Angular rates: [0.1, 0.1, 0.1]

**Input cost R** (diagonal):
- [1, 10, 10, 10] (thrust, roll, pitch, yaw moments)

### Input Constraints

```
u_min = [-0.5, -0.001, -0.001, -0.001]
u_max = [+0.5, +0.001, +0.001, +0.001]
```

## Usage

### In Webots

1. Open Webots
2. Load: `worlds/crazyflie_tinympc.wbt`
3. Run simulation

### Keyboard Controls

| Key | Action |
|-----|--------|
| Arrow Up/Down | Move target X |
| Arrow Left/Right | Move target Y |
| W/S | Adjust altitude |
| R | Reset to origin |
| SPACE | Print status |

## Dependencies

- TinyMPC: `pip install tinympc`
- NumPy: `pip install numpy`

**Note**: Use the `.venv_sim` environment to avoid dependency conflicts with cflib.

## Tuning Guide

### If drone oscillates:
- Decrease Q values (less aggressive position tracking)
- Increase R values (penalize control effort more)

### If drone responds slowly:
- Increase Q values for position
- Decrease R values
- Increase horizon N

### If drone drifts:
- Check A, B matrices match your model
- Verify sensor readings are correct
- Add integral action (modify state-space model)

## Model Details

### Continuous-time Dynamics (linearized at hover)

```
ṗx = vx
ṗy = vy
ṗz = vz
v̇x = g * theta    (small angle)
v̇y = -g * phi     (small angle)
v̇z = thrust_delta / mass
φ̇ = p
θ̇ = q
ψ̇ = r
ṗ = roll_moment / Ixx
q̇ = pitch_moment / Iyy
ṙ = yaw_moment / Izz
```

### CrazyFlie Parameters

| Parameter | Value | Unit |
|-----------|-------|------|
| Mass | 0.027 | kg |
| Ixx | 1.6e-5 | kg·m² |
| Iyy | 1.6e-5 | kg·m² |
| Izz | 2.9e-5 | kg·m² |
| g | 9.81 | m/s² |

## References

- [TinyMPC GitHub](https://github.com/TinyMPC/TinyMPC)
- [TinyMPC Documentation](https://tinympc.org/)
- [Bitcraze CrazyFlie](https://www.bitcraze.io/)
