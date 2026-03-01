# 🐍 Python Scripts

**Beach Autonomous Systems - ACC 2026**

---

## Purpose

This folder contains Python scripts for **environment setup only**.

These scripts use the `qvl` library to spawn actors (QCar, traffic lights, pedestrians, etc.) in QLabs. They do NOT control the vehicle or gather sensor data.

---

## ⚠️ Competition Compliance

> **Rule:** "Controlling the QCar or gathering data via the `qvl` library functions will invalidate any submission."

These scripts are **compliant** because they only:
- Spawn the QCar 2
- Set up traffic scenarios (pedestrians, other vehicles, lights, etc.)
- Configure the environment

All **autonomous vehicle control** and **sensor data gathering** is strictly handled by the `hal` and `pal` modules inside the `scenario_runner_python` directory. These Python spawn scripts are strictly for virtual environment instantiations.

---

## 📁 Scripts

| Script | Description |
|--------|-------------|
| `Setup_Real_Scenario_fullscale_x10.py` | Spawns QCar 2 and configures the environment for the detailed scenario runner |

---

## 🚀 Usage

Ensure the `scenario_runner_python` pip requirements are installed, then run the spawn environment generator.

### Run Spawn Script
```bash
python Setup_Real_Scenario_fullscale_x10.py
```

**Keep this script running** while your autonomous system algorithm in `scenario_runner_python/qcar2_detailed_scenario_runner.py` executes.

Press `Ctrl+C` when done to close the connection and despawn the entire environment.

---

## Workflow

1. **Launch QLabs**
2. **Select workspace** (Cityscape or specifically mapped ACC 2026 track)
3. **Run `python Setup_Real_Scenario_fullscale_x10.py`** - QCar and surroundings generate.
4. **Boot Autonomous Algorithms:** Inside a new terminal navigate to `scenario_runner_python` and run `qcar2_detailed_scenario_runner.py`
5. **Observe autonomous handling**
6. **Stop Algorithms `Ctrl+C`**
7. **Stop Spawn script `Ctrl+C`**

---

*Beach Autonomous Systems - CSULB*

