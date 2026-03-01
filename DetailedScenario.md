# 🎯 Detailed Scenario - ACC 2026 Self-Driving Car Competition

## Competition Objective

Teams will operate an **autonomous taxi service** in Quanser City. The goal is to **complete the Detailed Scenario** while adhering to traffic rules.

---

## 🚕 Taxi Service Operation

### Core Tasks

1. **Detailed Scenario**
   - System provides pick-up coordinates
   - System provides drop-off coordinates
   - Navigate efficiently

2. **Execute Rides**
   - Navigate to pick-up location
   - Pick up passenger
   - Navigate to drop-off location
   - Complete the ride safely

3. **Maximize Points**
   - Complete the scenario within the time limit
   - Avoid penalties that reduce points

---

## 🧠 Core Principles of Self-Driving

Based on the [Virtual Stage Competition Guide](https://quanser.github.io/student-competitions/events/common/Rules_and_Objectives/Virtual_Stage_Competition_Guide.html#core-principles-of-self-driving), the algorithm must demonstrate these four core principles:

### 1. Data Collection
A self-driving algorithm must collect and filter information from interoceptive and exteroceptive sensors. Demonstrating the conversion of raw data to meaningful information is critical for making higher-level decisions during an autonomous task.

**Implementation:**
- [x] Camera image processing (via `pal.QCarRealSense`)
- [x] LIDAR point cloud filtering (via `pal.QCarLidar`)
- [-] Sensor fusion techniques
- [x] Data preprocessing pipelines

### 2. Interpretation
Using system-relevant data, the car must correlate gathered information to factors happening internally or externally in the environment.

**External Factors:**
- [x] Traffic sign identification (YOLO)
- [x] Traffic light recognition (YOLO)
- [x] Pedestrian detection
- [x] Other vehicle detection

**Internal Factors:**
- [x] System state identification
- [x] Error monitoring
- [ ] Performance tracking

### 3. Control Systems
From the set of viable options determined in interpretation, the car must execute accurately on the chosen option.

**Requirements:**
- [x] Staying within lanes (Pure Pursuit & OpenCV Edge Detection)
- [x] Executing turns
- [x] Stopping at traffic controls (State Machine logic)
- [x] Altering path based on obstacles (LIDAR)
- [x] Maintaining desired speed

### 4. Localization and Path Planning
The car must understand where it is within the roadmap and determine how to get to another location.

**Requirements:**
- [x] Global/local map storage (`waypoints.txt` coordinates)
- [x] Position determination in space (Simulation socket `_QLabsWorldTransform`)
- [x] Route planning to destinations
- [ ] Dynamic route adjustment based on:
  - Vehicles on the road
  - Road obstructions
  - Pedestrians entering/leaving roadway

---

## 🚦 Traffic Scenarios

The autonomous vehicle must handle the following traffic scenarios:

### Basic Navigation
- [-] Lane following
- [x] Intersection handling
- [x] Turn execution (left, right, U-turns)
- [-] Speed limit compliance

### Traffic Control
- [x] Traffic light recognition and compliance
- [x] Stop sign detection and compliance
- [x] Yield sign handling
- [ ] Pedestrian crosswalk handling

### Dynamic Obstacles
- [-] Other vehicle detection and avoidance
- [-] Pedestrian detection and yielding
- [ ] Construction zones
- [ ] Emergency vehicle handling

### Advanced Scenarios
- [ ] Merging and lane changes
- [x] Roundabout navigation
- [x] Parking at pick-up/drop-off locations

---

## 📊 Ranking Criteria

Teams will be ranked using the following criteria (from [Virtual Stage Competition Guide](https://quanser.github.io/student-competitions/events/common/Rules_and_Objectives/Virtual_Stage_Competition_Guide.html)):

| Priority | Criteria | Description |
|----------|----------|-------------|
| 1 | **Algorithm Readiness** | Based on the four core principles of self-driving |
| 2 | **Driving Accuracy** | Staying within the lanes |
| 3 | **Traffic Compliance** | Timely reaction to road signage and traffic controls |
| 4 | **Communication** | Clear and concise explanation of self-driving concepts |

> ⚠️ **Important:** Clear communication is one of the most important criteria because it shows the judges how well the team understands the principles of self-driving.

---

## 📹 Virtual Stage Submission Requirements

Based on the [Virtual Stage Competition Guide](https://quanser.github.io/student-competitions/events/common/Rules_and_Objectives/Virtual_Stage_Competition_Guide.html):

### Submission Checklist
- [x] **Video:** Maximum **3-minute** demonstration of self-driving capabilities
- [x] **Software:** GitHub link to repository with submission code
- [x] **Video Link:** YouTube link demonstrating the code

### ⚠️ Critical Rule
> **Controlling the QCar or gathering data via the `qvl` library functions will invalidate any submission.**

The QCar must be controlled through proper autonomous algorithms, not manual control or scripted paths using qvl.

### Video Content Requirements
1. Self-driving capabilities demonstration
2. Explanation of approach and algorithms
3. Show the four core principles in action

---

## 🗺️ Coordinate System

The coordinate system is consistent for both virtual and physical stages. QLabs contains 1:1 representations of the Quanser Roadmaps.

- **Origin:** `[0, 0, 0]` defined by the coordinate tool in QLabs
- **Base Frame:** All coordinates determined from the origin
- **CityScape Maps:** Full-scale versions of Physical Quanser Roadmaps

---

## 🗺️ Quanser City Environment

### City Features
- Multiple lanes and road types
- Traffic lights and stop signs
- Pedestrian crossings
- Intersections (4-way, T-junctions)
- Pick-up/drop-off zones

### Sensor Suite (QCar 2)
- **Cameras:** Front, rear, and side cameras for visual perception
- **LIDAR:** 360° point cloud for obstacle detection
- **IMU:** Inertial measurement for vehicle state
- **Encoders:** Wheel speed and position feedback
- **GPS:** Position estimation (simulated)

---

## 🏁 Success Criteria

### Minimum Requirements
- [x] Vehicle can spawn and initialize in QLabs
- [x] Basic lane following capability
- [x] Stop at traffic lights and stop signs
- [x] Complete at least one full ride

### Competitive Requirements
- [x] Reliable perception across scenarios
- [x] Smooth path planning
- [x] Efficient route optimization
- [x] High ride completion rate
- [ ] Minimal penalties

### Excellence Criteria
- [-] Robust handling of edge cases
- [x] Adaptive behavior
- [x] Innovative approaches

---

## 📚 References

- [Virtual Stage Competition Guide](https://quanser.github.io/student-competitions/events/common/Rules_and_Objectives/Virtual_Stage_Competition_Guide.html)
- [ROS Technical Resources](https://github.com/quanser/student-competition-resources-ros)
- [QLabs Documentation](https://docs.quanser.com/qlabs/)
- [Competition Page](https://quanser.github.io/student-competitions/events/acc-2026/index.html)

---

## 🎯 Team Goals - Beach Autonomous Systems

### Phase 1: Foundation (January 2026)
- [x] Complete environment setup
- [x] Understand QLabs and QCar 2 interface
- [x] Basic vehicle control working

### Phase 2: Perception (January-February 2026)
- [x] Camera processing pipeline (OpenCV Canny edge detection)
- [x] LIDAR processing (Raycast obstacle detection)
- [x] Object detection (PyTorch/YOLO for signs and lights)

### Phase 3: Planning & Control (February 2026)
- [x] Path planning implementation (Waypoint interpolation)
- [x] Decision-making logic (Yield/Stop state machines)
- [x] Control tuning (hal.PurePursuitController)

### Phase 4: Integration (February 2026)
- [x] Full system integration
- [x] Testing and debugging
- [x] Video preparation and submission

---

*Last Updated: March 2026*  
*Beach Autonomous Systems - CSULB*
