# 🎯 Detailed Scenario - ACC 2026 Self-Driving Car Competition

## Competition Objective

Teams will operate an **autonomous taxi service** in Quanser City. The goal is to **maximize profits** by successfully completing passenger rides while adhering to traffic rules and maintaining high customer satisfaction ratings.

---

## 🚕 Taxi Service Operation

### Core Tasks

1. **Receive Ride Requests**
   - System provides pick-up coordinates
   - System provides drop-off coordinates
   - Navigate efficiently to maximize earnings

2. **Execute Rides**
   - Navigate to pick-up location
   - Pick up passenger
   - Navigate to drop-off location
   - Complete the ride safely

3. **Maximize Profit**
   - Complete as many rides as possible within the time limit
   - Earn higher ratings for better payouts
   - Avoid penalties that reduce earnings

---

## 🚦 Traffic Scenarios

Your autonomous vehicle must handle the following traffic scenarios:

### Basic Navigation
- [ ] Lane following
- [ ] Intersection handling
- [ ] Turn execution (left, right, U-turns)
- [ ] Speed limit compliance

### Traffic Control
- [ ] Traffic light recognition and compliance
- [ ] Stop sign detection and compliance
- [ ] Yield sign handling
- [ ] Pedestrian crosswalk handling

### Dynamic Obstacles
- [ ] Other vehicle detection and avoidance
- [ ] Pedestrian detection and yielding
- [ ] Construction zones
- [ ] Emergency vehicle handling

### Advanced Scenarios
- [ ] Merging and lane changes
- [ ] Roundabout navigation
- [ ] Parking at pick-up/drop-off locations
- [ ] Weather conditions 

---

## 📊 Scoring System

### Rating Factors

| Factor | Impact | Description |
|--------|--------|-------------|
| **Ride Completion** | High | Successfully completing the ride |
| **Safety** | High | No collisions, safe driving behavior |
| **Rule Compliance** | Medium | Following traffic rules |
| **Efficiency** | Medium | Time to complete ride |
| **Comfort** | Low | Smooth driving, minimal jerks |

### Penalties

| Violation | Penalty |
|-----------|---------|
| Collision | Major deduction |
| Running red light | Significant deduction |
| Running stop sign | Significant deduction |
| Speeding | Moderate deduction |
| Leaving lane | Minor deduction |
| Excessive braking | Minor deduction |

### Earnings Formula
```
Ride Earnings = Base Fare × Rating Multiplier - Penalties
```

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

## 📋 Virtual Stage Requirements

### Video Submission Criteria

Your video should demonstrate:

1. **Technical Capability**
   - Vehicle spawning and initialization
   - Basic navigation (lane following, turns)
   - Traffic rule compliance
   - Obstacle handling

2. **Algorithm Readiness**
   - Perception pipeline
   - Planning and decision making
   - Control implementation
   - Integration and testing

3. **Documentation**
   - Clear explanation of approach
   - Architecture overview
   - Team presentation

### Recommended Video Structure
1. Introduction 
2. System architecture overview
3. Live demonstration in QLabs
4. Handling of specific scenarios
5. Conclusion and next steps

---

## 🏁 Success Criteria

### Minimum Requirements
- [ ] Vehicle can spawn and initialize in QLabs
- [ ] Basic lane following capability
- [ ] Stop at traffic lights and stop signs
- [ ] Complete at least one full ride

### Competitive Requirements
- [ ] Reliable perception across scenarios
- [ ] Smooth path planning
- [ ] Efficient route optimization
- [ ] High ride completion rate
- [ ] Minimal penalties

### Excellence Criteria
- [ ] Robust handling of edge cases
- [ ] Adaptive behavior
- [ ] High customer ratings
- [ ] Innovative approaches

---

## 📚 Reference Materials

- [ROS Technical Resources](https://github.com/quanser/student-competition-resources-ros)
- [QLabs Documentation](https://docs.quanser.com/qlabs/)
- [ACC 2025 Reference](https://github.com/quanser/ACC-Competition-2025)
- [Competition Page](https://quanser.github.io/student-competitions/events/acc-2026/index.html)

---

## 🎯 Team Goals - Beach Autonomous Systems

### Phase 1: Foundation (January 2026)
- [ ] Complete environment setup
- [ ] Understand QLabs and QCar 2 interface
- [ ] Basic vehicle control working

### Phase 2: Perception (January-February 2026)
- [ ] Camera processing pipeline
- [ ] LIDAR processing
- [ ] Object detection (signs, lights, obstacles)

### Phase 3: Planning & Control (February 2026)
- [ ] Path planning implementation
- [ ] Decision-making logic
- [ ] Control tuning

### Phase 4: Integration (February 2026)
- [ ] Full system integration
- [ ] Testing and debugging
- [ ] Video preparation and submission

---

*Last Updated: January 2026*  
*Beach Autonomous Systems - CSULB*

