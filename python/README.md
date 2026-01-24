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
- Set up traffic scenarios (pedestrians, other vehicles, etc.)
- Configure the environment

All **vehicle control** and **sensor data gathering** is done via **QUARC/Simulink**.

---

## 📁 Scripts

| Script | Description |
|--------|-------------|
| `Setup_Real_Scenario_fullscale_x10.py` | Spawns QCar 2 at default position for QUARC control |

---

## 🚀 Usage

### Install Requirements



### Run Spawn Script
```bash
python Setup_Real_Scenario_fullscale_x10.py
```

**Keep this script running** while using your Simulink model.

Press `Ctrl+C` when done to close connection and despawn the QCar.

---

## Workflow

1. **Launch QLabs** (from MATLAB or manually)
2. **Select workspace** (Cityscape or Open Road)
3. **Run `Setup_Real_Scenario_fullscale_x10.py`** - QCar appears in QLabs
4. **Run Simulink model** - Controls the spawned QCar via QUARC
5. **Stop Simulink** when done
6. **Ctrl+C** the Python script

---

*Beach Autonomous Systems - CSULB*

