# 🚗 Beach Autonomous Systems - ACC 2026 Self-Driving Car Competition

<div align="center">

![Competition Banner](https://img.shields.io/badge/ACC%202026-Self--Driving%20Car%20Competition-blue?style=for-the-badge)
![Team](https://img.shields.io/badge/Team-Beach%20Autonomous%20Systems-gold?style=for-the-badge)
![University](https://img.shields.io/badge/CSULB-Go%20Beach!-black?style=for-the-badge)

**California State University Long Beach**  
*Department of Computer Engineering and Computer Science*

</div>

---

## 🏆 Competition Overview

This competition involves creating a self-driving algorithm capable of navigating through **Quanser City** as an autonomous taxi service. The objective is to **complete the Detailed Scenario** within a certain time:

- Navigating to selected pick-up and drop-off coordinates
- Handling various traffic scenarios while adhering to rules of the road
- Based on ride performance
- Completing scenario within the timeframe

## 👥 Team Information

| Role | Name | Email |
|------|------|-------|
| **Team Captain** | Ricardo Cervantes | ricardo.cervantes01@student.csulb.edu |
| **Team Member** | Michelle Do | Michelle.Do01@student.csulb.edu |
| **Team Member** | Gricel Aguilar Quiroz | Gricel.AguilarQuiroz01@student.csulb.edu |
| **Team Member** | Delsin Carbonell | Delsin.Carbonell01@student.csulb.edu |
| **Team Member** | Matthew Margulies | Matthew.Margulies01@student.csulb.edu |
| **Team Member** | Ishank Sharma | Ishank.Sharma01@student.csulb.edu |
| **Team Member** | Kiki Han | Kiki.Han01@student.csulb.edu |
| **Faculty Supervisor** | Dr. Xin Qin | xin.qin@csulb.edu |

**Institution:** California State University Long Beach  
**Department:** Computer Engineering and Computer Science  
**Team Name:** Beach Autonomous Systems

---

## 📅 Competition Timeline

| Date | Event | Description |
|------|-------|-------------|
| Nov 18, 2025 | Registration Opens | ✅ Completed |
| Dec 1, 2025 | Technical Resources Released | ROS resources available for development |
| Dec 1 - Feb 27, 2026 | **Virtual Stage** | Develop self-driving algorithms in QLabs |
| March 2-7, 2026 | Virtual Stage Evaluation | Judges review video submissions |
| March 9 - May 22, 2026 | Physical Implementation | Top teams receive QCar 2 hardware |
| May 25-28, 2026 | **Physical Stage @ ACC** | Live competition in New Orleans, LA |

---

## 🏗️ Competition Stages

### Stage 1: Virtual Design and Submission
- Develop self-driving algorithms in Quanser's virtual environment (QLabs)
- Create a video submission demonstrating algorithm capabilities
- **Deadline:** March 1, 2026

### Stage 2: Physical Implementation
- Top 6 teams receive physical QCar 2 from Quanser
- Implement algorithms on real hardware
- **Duration:** March 9 - May 22, 2026

### Stage 3: On-Site Competition
- Bring QCar 2 to ACC conference venue
- Compete live against other selected teams
- **Location:** New Orleans, Louisiana, USA
- **Date:** May 25-28, 2026

---

## 💻 Software Stack

The project uses a pure **Python-based** architecture, leveraging Quanser's `hal` and `pal` modules alongside PyTorch for real-time perception, localization, and control.

### Why Pure Python?
- **Native Integration:** Direct interaction with Quanser's Hardware Abstraction Layer (`hal`) and Platform Abstraction Layer (`pal`).
- **Advanced Perception:** Seamless deployment of PyTorch (`torch`) and Ultralytics YOLO (`best.torchscript`) models for high-performance object detection.
- **Unified Pipeline:** Perception, planning, and control reside in a single executable loop (`qcar2_detailed_scenario_runner.py`), eliminating the need to bridge between Simulink block diagrams and Python scripts.

---

## 🔗 Official Resources

### Competition Documentation
- 📋 [ACC 2026 Competition Page](https://quanser.github.io/student-competitions/events/acc-2026/index.html)
- 📚 [Virtual Stage Competition Guide](https://quanser.github.io/student-competitions/events/acc-2026/index.html)
- 🔧 [ROS Technical Resources](https://github.com/quanser/student-competition-resources-ros)
- 🎓 [ACC 2026 Conference](https://acc2026.a2c2.org/)

### Technical Documentation
- 🖥️ [Quanser Interactive Labs Support](https://www.quanser.com/qil-support/)
- 📖 [QLabs Documentation](https://docs.quanser.com/qlabs/)
- 🐍 [Quanser Python API](https://docs.quanser.com/python/)
- ⚙️ [Quanser C API](https://docs.quanser.com/c/)

---

## 🚀 Environment Setup

### 1. Requirements
- **OS:** Windows 10/11
- **Python:** Python 3.9+ (Recommended)
- **Quanser Interactive Labs (QLabs):** [Download QLabs](https://www.quanser.com/digital/quanser-interactive-labs/)

### 2. Install Dependencies
Set up the Python environment using the requirements defined in the `scenario_runner_python` module:
```bash
# Navigate to the runner directory
cd scenario_runner_python

# Create a virtual environment (optional but recommended)
python -m venv venv
venv\Scripts\activate

# Install strictly defined requirements
pip install -r requirements.txt
```
*Note: Ensure to install the correct `torch` (PyTorch) wheels compatible with your hardware (CUDA or CPU only) as this affects the YOLO object detection speed.*

### 3. Launch QLabs
Launch the QLabs software and select the **Cityscape**/**Cityscape-lite** workspace.

### 4. Spawn the QCar 2 Environment
Open a terminal and set up the interactive actors (traffic lights, cameras, and vehicle):
```bash
cd python
python Setup_Real_Scenario_fullscale_x10.py
```

### 5. Run the Autonomous Pipeline
Open a new terminal and execute the primary autonomous loop. This will connect to the spawned QCar and utilize the `PurePursuitController` with YOLO perception:
```bash
cd scenario_runner_python
python qcar2_detailed_scenario_runner.py
```

---

## 📁 Project Structure

```text
ACC-Competition-2026/
├── README.md                 # This file
├── DetailedScenario.md       # Scenario objectives & algorithms overview
├── python/                   # Scripts for QLabs simulation setup
│   ├── Setup_Real_Scenario_fullscale_x10.py  # Main spawn script
│   └── spawn_qcar.py         # Legacy script for generating base QCar
├── scenario_runner_python/   # Core autonomous execution pipeline
│   ├── best.torchscript      # Pre-trained YOLO model
│   ├── qcar2_detailed_scenario_runner.py     # Main execution script
│   ├── requirements.txt      # Python dependencies
│   ├── waypoints.txt         # Pre-defined path coordinates
│   ├── hal/                  # Hardware Abstraction Layer
│   ├── pal/                  # Platform Abstraction Layer
│   ├── pit/                  # Perception Image Tools 
│   └── tools/                # Waypoint/Map utilities
└── docs/                     # Documentation updates & architecture
    ├── architecture.md
    └── progress/
```

---

## ❓ FAQ & Support

- **Technical Questions:** Post on [ROS Technical Resources Issues](https://github.com/quanser/student-competition-resources-ros/issues)
- **Competition Inquiries:** Email studentcompetition@quanser.com
- **Discussions:** Check the [Discussions Tab](https://github.com/quanser/student-competition-resources-ros/discussions)

---

## 📝 Development Log

| Date | Milestone | Status |
|------|-----------|--------|
| Jan 2026 | Project Setup | 🟢 Complete |
| Jan 2026 | Environment Configuration | 🟢 Complete |
| Feb 2026 | Basic Navigation | 🟢 Complete |
| Feb 2026 | Traffic Handling | 🟢 Complete |
| Feb 2026 | Video Submission | 🟢 Complete |

---

<div align="center">

**Go Beach! 🏖️**

*Beach Autonomous Systems - CSULB CECS*

</div>
