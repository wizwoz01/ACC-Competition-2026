# 🟦 MATLAB Development

---

## 📁 Folder Structure

```
matlab/
├── models/           # Simulink models
├── scripts/          # MATLAB scripts
├── functions/        # Reusable functions
└── README.md         # This file
```

---

## 🚀 Quick Start (QUARC + QLabs)

### 1. Launch QLabs
```matlab
QLabs.launch
```

### 2. Select Workspace in QLabs GUI
- Choose **Cityscape** or **Open Road** workspace
- Wait for the environment to load fully (buildings/road appear)

### 3. ⚠️ Spawn the QCar 2 (CRITICAL STEP!)

**Open World workspaces do NOT auto-spawn the QCar!** You must run the Python spawn script:

```bash
cd python
python Setup_Real_Scenario_fullscale_x10.py
```

Keep this script running while using Simulink. The QCar will appear in QLabs.

> **Competition Note:** Using `qvl` for spawning is allowed. The rule forbids using `qvl` for *controlling* or *gathering data*, not for environment setup.

### 4. Verify Connection (Optional)
```matlab
run('matlab/scripts/test_qlabs_connection.m')
```
All ports should show [OK] now that the QCar is spawned.

### 5. Open Your Simulink Model
Your Simulink model should contain:
- **HIL Initialize** block with:
  - Board type: `qcar2`
  - Board identifier: `0@tcpip://localhost:18960`

### 6. Run the Simulink Model
- Click **Run** in Simulink
- The virtual QCar 2 will respond to your control commands
- Monitor in QLabs window

### 7. Stop When Done
- Stop the Simulink model
- Press Ctrl+C in Python spawn script
- Close QLabs

---

## 🚕 Taxi Scenario (Virtual Detailed Scenario)

Implements the official sequence:

- Start at Taxi Hub, LED **Magenta**
- Navigate to pickup `[0.125, 4.395]`, LED **Green**
- Full stop, LED **Blue** (pickup)
- Navigate to dropoff `[-0.905, 0.800]`, LED **Green**
- Full stop, LED **Orange** (dropoff)
- Return to Taxi Hub, LED **Magenta** (await)

### Run (recommended)

1. In a terminal, run the environment setup/spawn script:
  - `python/Setup_Real_Scenario.py`
    or
  - `python/Setup_Real_Scenario_fullscale_x10.py`
2. In MATLAB:
  - Run `matlab/scripts/run_taxi_scenario.m`

This generates an algorithm-only model at:

- `matlab/models/taxi_stack.slx`

That model outputs:


Wire its inputs/outputs into your QUARC I/O model (GPS/IMU in, motor/steering/LED out).

### Parameters

The scenario coordinates and thresholds are in:

- `matlab/config/taxi_scenario_params.m`

---

## 📂 What Goes Where

### `models/` - Simulink Models
- `vehicle_control.slx` - Main vehicle controller with HIL blocks
- `path_planning.slx` - Path planning system
- `perception.slx` - Sensor processing pipeline

### `scripts/` - MATLAB Scripts
- `main.m` - Main entry point
- `test_*.m` - Test scripts

### `functions/` - Reusable Functions
- `pid_controller.m` - PID control function
- `pure_pursuit.m` - Path following algorithm
- `process_lidar.m` - LIDAR processing
- `detect_lanes.m` - Lane detection

---

## 🔌 QLabs Port Reference (QCar 2)

| Port Type | Port Number | Board Identifier / URI |
|-----------|-------------|------------------------|
| **HIL (Control)** | 18960 | `0@tcpip://localhost:18960` |
| **Camera Front** | 18942 | `0@tcpip://localhost:18942` |
| **Camera Right** | 18940 | `0@tcpip://localhost:18940` |
| **Camera Back** | 18941 | `0@tcpip://localhost:18941` |
| **Camera Left** | 18943 | `0@tcpip://localhost:18943` |
| **RGBD Camera** | 18965 | `0@tcpip://localhost:18965` |
| **Lidar** | 18966 | `tcpip://localhost:18966` |
| **GPS** | 18967 | `tcpip://localhost:18967` |
| **LED Strip** | 18969 | `tcpip://localhost:18969` |

---

## 🎮 QUARC Simulink Blocks

### HIL Initialize Block
```
Path: QUARC Targets -> Data Acquisition -> Generic -> Configuration
Board type: qcar2
Board identifier: 0@tcpip://localhost:18960
```

### HIL Read/Write Blocks
```
Path: QUARC Targets -> Data Acquisition -> Generic -> Immediate I/O
- Motor throttle: PWM channel 1000
- Steering: Other output channel 0
```

### Video Capture Block (Cameras)
```
Path: QUARC Targets -> Multimedia -> Video Capture
Device: 0@tcpip://localhost:18942  (for front camera)
```

### Video3D Capture Block (RGBD)
```
Path: QUARC Targets -> Multimedia -> Video3D Capture
Device: 0@tcpip://localhost:18965
```

---

## 📝 File Naming Convention

| Type | Pattern | Example |
|------|---------|---------|
| Scripts | `setup_qcar.m` |
| Functions | `pid_controller.m` |
| Models | `vehicle_control.slx` |
| Test files | `test_*.m` | `test_steering.m` |
| Data files | `*.mat` | `calibration_data.mat` |

---

## ⚠️ Notes

- **Always start QLabs BEFORE running your Simulink model**
- Make sure the workspace (Cityscape/Open Road) is fully loaded
- Check CPS in QLabs settings if things are running slow
- Stop the Simulink model before closing QLabs
- **Same Simulink model works for both virtual (QLabs) and physical (QCar 2 hardware)**

---

## 🚗 Virtual → Physical Transition

To switch from virtual to physical hardware:
1. Change Board identifier from `0@tcpip://localhost:18960` to `0`
2. Set Simulink target to **QUARC Linux QCar 2 Target**
3. Deploy to the physical QCar 2

---

## ⚠️ Competition Rule Compliance

> **"Controlling the QCar or gathering data via the `qvl` library functions will invalidate any submission."**

This setup uses **QUARC HIL blocks**, NOT the qvl library. Our autonomous algorithm makes all driving decisions through the Simulink control system.

---

*Beach Autonomous Systems - CSULB*
