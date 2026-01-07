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

This competition involves creating a self-driving algorithm capable of navigating through **Quanser City** as an autonomous taxi service. The objective is to **maximize profits** within a certain time period by:

- Navigating to selected pick-up and drop-off coordinates
- Handling various traffic scenarios while adhering to rules of the road
- Earning ratings based on ride performance
- Completing as many profitable rides as possible within the timeframe

## 👥 Team Information

| Role | Name | Email |
|------|------|-------|
| **Team Captain** | Ricardo Cervantes | ricardo.cervantes01@student.csulb.edu |
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
- **Deadline:** February 27, 2026

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

Using **MATLAB/Simulink** for this competition. Here's why:

### 🔀 Development Pathways Comparison

| | **MATLAB/Simulink** ✅ | **ROS 2 Humble** |
|---|---------------------|------------------|
| **Best For** | Control systems, rapid prototyping | Robotics, multi-sensor fusion |
| **OS** | Windows 10/11 (native) | Ubuntu 24.04 / WSL2 |
| **GPU Required** | ❌ No | ✅ Yes (NVIDIA) |
| **Setup Complexity** | ⭐ Simple | ⭐⭐⭐ Complex |
| **Docker Required** | ❌ No | ✅ Yes |

---

### 🟦 MATLAB Setup

#### Requirements
- **OS:** Windows 10/11 (native) - Using a ThinkPad L13
- **Software:** MATLAB R2023a+ with Simulink
- **Add-on:** Quanser Interactive Labs for MATLAB
- **Simulation:** Quanser Interactive Labs (QLabs)

#### Development Structure
```
├── matlab/                    # MATLAB/Simulink files
│   ├── models/               # Simulink models
│   │   ├── vehicle_control.slx
│   │   ├── path_planning.slx
│   │   └── perception.slx
│   ├── scripts/              # MATLAB scripts
│   │   ├── main.m
│   │   ├── setup_qcar.m
│   │   └── utils/
│   └── config/               # Configuration files
├── Software/                 # Development guides
├── Handbook/                 # Competition rules
└── docs/                     # Documentation
```

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

### 1. Install QLabs
- Download from [Quanser Interactive Labs](https://www.quanser.com/digital/quanser-interactive-labs/)
- Register at [Quanser Academic Portal](https://portal.quanser.com/Accounts/Register)
- Make sure to register for the competition to get QLabs access

### 2. Install MATLAB Add-on
```matlab
% In MATLAB Command Window:
% Go to Add-Ons → Get Add-Ons → Search "Quanser Interactive Labs for MATLAB"
% Click "Add" to install
```

### 3. Set Up QLabs Connection
```matlab
QLabs.setup      % First-time setup
QLabs.launch     % Launch QLabs from MATLAB
```

### 4. Start Developing!
- Create Simulink models for vehicle control
- Use MATLAB scripts for path planning algorithms
- Test everything in the QLabs simulation environment
- See `Software/Development_Guide.md` for full workflow

---

## 📁 Project Structure

```
ACC-Competition-2026/
├── README.md                 # This file
├── DetailedScenario.md       # Competition scenario and objectives
├── matlab/                   # MATLAB/Simulink development
│   ├── models/              # Simulink models
│   ├── scripts/             # MATLAB scripts
│   └── functions/           # Reusable functions
├── Software/                 # Development guides
│   ├── README.md
│   └── Development_Guide.md
├── Handbook/                 # Competition rules
│   └── README.md
├── docs/                     # Documentation
│   ├── architecture.md
│   └── progress/
└── scripts/                  # Utility scripts
    └── setup.sh
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
| Jan 2026 | Project Setup | 🟢 In Progress |
| Jan 2026 | Environment Configuration | 🟢 In Progress |
| Feb 2026 | Basic Navigation | ⏳ Pending |
| Feb 2026 | Traffic Handling | ⏳ Pending |
| Feb 2026 | Video Submission | ⏳ Pending |

---

<div align="center">

**Go Beach! 🏖️**

*Beach Autonomous Systems - CSULB CECS*

</div>
