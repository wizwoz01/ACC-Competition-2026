# 🗺️ Planning Package

This package handles path planning and decision making.

## Responsibilities

- Global path planning (route to destination)
- Local path planning (obstacle avoidance)
- Behavior planning (traffic rules, decisions)
- Mission planning (ride management)

## Nodes

| Node | Description | Topics |
|------|-------------|--------|
| `global_planner` | Plans route to destination | `/goal` → `/planning/global_path` |
| `local_planner` | Plans local trajectory | `/perception/objects` → `/planning/local_path` |
| `behavior_planner` | Decides vehicle behavior | `/perception/*` → `/planning/behavior` |
| `mission_planner` | Manages ride requests | `/taxi/request` → `/taxi/status` |

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Mission Planner                       │
│              (Ride requests, destinations)               │
└─────────────────────┬───────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────┐
│                    Global Planner                        │
│              (Route planning, waypoints)                 │
└─────────────────────┬───────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────┐
│                   Behavior Planner                       │
│         (Traffic rules, state machine, decisions)        │
└─────────────────────┬───────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────┐
│                    Local Planner                         │
│            (Trajectory generation, avoidance)            │
└─────────────────────────────────────────────────────────┘
```

## State Machine

```
┌────────────┐     ┌──────────────┐     ┌────────────┐
│   IDLE     │────▶│   DRIVING    │────▶│  STOPPED   │
└────────────┘     └──────────────┘     └────────────┘
      ▲                   │                    │
      │                   ▼                    │
      │            ┌──────────────┐            │
      └────────────│   TURNING    │◀───────────┘
                   └──────────────┘
```

## Usage

```bash
# Run planning nodes
ros2 launch planning planning_launch.py
```

---

*Beach Autonomous Systems - CSULB*

