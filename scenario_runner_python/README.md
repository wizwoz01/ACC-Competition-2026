# Scenario Runner 
The core autonomous pipeline replacing all Legacy MATLAB/Simulink workflows. This execution node encompasses the decision logic (State Machines), vehicle sensor acquisition (`pal`), motion control (`hal`), and visual perception (PyTorch + OpenCV).

## Included files
- `qcar2_detailed_scenario_runner.py` : Main execution state machine and algorithm loop.
- `waypoints.txt` : The absolute coordinates generated for the Quanser custom track.
- `sign_labels.txt` : Label maps for YOLO output indexing (e.g. Traffic Lights, Stop Signs).
- `best.torchscript` : Compiled YOLO detection PyTorch model optimized for live predictions.
- `requirements.txt` : Pip environment declarations.
- `hal/` & `pal/` : Hardware/Platform Abstraction Layers containing `PurePursuitController` and hardware interfaces (`QCarLidar`, `QCarRealSense`).
- `pit/` : Perception/Image tools encompassing Edge/Lane detection networks.

## Prerequisites
1. Python 3.9+ installed and configured.
2. Dependencies installed via `pip install -r requirements.txt`. (Refer to the root [README](../README.md) for full setup instructions).
3. The QLabs simulator spawning script running in the `../python/` folder.

## Autonomous Workflow Architecture
The `qcar2_detailed_scenario_runner.py` drives the QCar 2 using the following logic loops:
- **State Machine Control:** Evaluates global states (driving, yielding at stop signs, waiting at lights, cross-track error recovery).
- **Perception Pipeline:**
  - RealSense Camera captures RGB inputs.
  - Image is passed through the `best.torchscript` YOLO model to identify relevant bounding boxes (signs/lights).
  - OpenCV isolates road edges (yellow/white masks) to maintain Lane Centering overrides.
- **LIDAR Safety:** Queries 360-degree point clouds filtering for obstacles strictly within the vehicle's forward path trajectory.
- **Hardware Abstraction Layer (HAL):** `PurePursuitController` dynamically traces the absolute map coordinates located in `waypoints.txt` using localization data polled via a custom simulator socket `_QLabsWorldTransform()`.

## Run
Ensure the virtual environment is activated and the spawning script is already active in another terminal.

Run directly in PowerShell/CMD:
```powershell
python .\qcar2_detailed_scenario_runner.py --waypoints .\waypoints.txt --sign-model .\best.torchscript --sign-labels .\sign_labels.txt
```

*(Note: Provide `--sign-device cpu` if a GPU is unavailable, though GPU integration via CUDA is natively supported and highly recommended for minimal latency).*
