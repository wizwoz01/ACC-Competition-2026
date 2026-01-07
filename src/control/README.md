# 🎮 Control Package

This package handles vehicle control commands.

## Responsibilities

- Steering control
- Throttle/brake control
- Path following
- Low-level control loops

## Nodes

| Node | Description | Topics |
|------|-------------|--------|
| `vehicle_controller` | Main control loop | `/planning/local_path` → `/cmd_vel` |
| `pid_controller` | PID control for steering/speed | `/control/setpoint` → `/control/output` |
| `path_follower` | Pure pursuit / Stanley controller | `/planning/path` → `/control/steering` |

## Control Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   Path Follower                          │
│           (Pure Pursuit / Stanley Controller)            │
└─────────────────────┬───────────────────────────────────┘
                      │
          ┌───────────┴───────────┐
          ▼                       ▼
┌─────────────────┐     ┌─────────────────┐
│ Steering PID    │     │  Speed PID      │
└────────┬────────┘     └────────┬────────┘
         │                       │
         ▼                       ▼
┌─────────────────────────────────────────┐
│           Vehicle Interface             │
│        (QCar 2 Commands)                │
└─────────────────────────────────────────┘
```

## Control Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `max_speed` | Maximum speed (m/s) | 1.0 |
| `max_steering` | Maximum steering angle (rad) | 0.5 |
| `lookahead_distance` | Pure pursuit lookahead | 0.5 |
| `kp_steering` | Steering P gain | 1.0 |
| `kd_steering` | Steering D gain | 0.1 |

## Usage

```bash
# Run control nodes
ros2 launch control control_launch.py
```

## Tuning Tips

1. Start with low gains
2. Increase P until oscillation
3. Add D to dampen oscillation
4. Test at various speeds

---

*Beach Autonomous Systems - CSULB*

