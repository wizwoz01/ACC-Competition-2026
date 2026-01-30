# QCar2 Detailed Scenario Model
**Working Concept – In Progress**

## Overview

This Simulink model implements an early working concept for autonomous driving with the **Quanser QCar 2**, combining:

- Waypoint-based navigation
- Vision-based lane detection
- A supervisory state machine for behavior switching
- Speed and steering control
- Basic odometry and motion estimation

The model is **not complete** and is under active development. Current functionality focuses on validating subsystem integration and state transitions rather than robust driving performance.

---

## Current Status

### ✅ What Works
- State machine successfully switches modes  
  - Waypoint following → Lane following
- Waypoint follower makes partial progress toward pickup locations
- Lane detection produces a stable `laneStable` signal
- Core control loops (speed, steering, odometry) are integrated
- End-to-end simulation runs reliably

### ⚠️ Known Issues
- Vehicle does not reliably stay on the road
- Lane-following lacks strong lateral containment
- Vehicle may become stuck or oscillatory
- Waypoint follower can stall when off-path
- No recovery logic when perception degrades

---

## High-Level Architecture

Perception
- Lane Detection (HSV thresholding + area filtering)
- Lane Stability Debounce
- Basic Depth Awareness

Planning
- Path Translation
- Pure Pursuit Waypoint Follower
- Mode Selection

Control
- Speed Controller
- Steering Command Generation
- Turn-Speed Handling

Supervision
- TaxiHubSupervisor (Stateflow)

Estimation
- Odometry
- Speed Estimation

---

## TaxiHubSupervisor (State Machine)

### States
- INIT – System initialization
- ARMING – Motor enable delay
- WaypointFollowing – Pure pursuit control
- SEARCH_LANE – Slow forward motion to reacquire lane
- LANE_FOLLOW – Vision-based lane following

### Key Transitions
- WaypointFollowing → SEARCH_LANE when waypoint reached
- SEARCH_LANE → LANE_FOLLOW when `laneStable == true`

---

## Lane Detection

Lane detection uses HSV color thresholding and binary mask processing.

Pipeline:
1. RGB → HSV conversion
2. Color thresholding
3. Binary filtering
4. Region-of-interest masking
5. Mask area calculation
6. Debounce to generate `laneStable`

At present, detection indicates **lane presence only**, not lane geometry.

---

## Waypoint Following

A segment-based Pure Pursuit controller is used.

Features:
- Adaptive lookahead
- Lateral-error-based speed limiting
- Index freeze when far from path
- Bounded waypoint progression

Limitations:
- Waypoints are not lane-constrained
- No replanning or recovery behaviors

---

## Odometry

A kinematic bicycle model with Euler integration is used.
Drift is expected and currently unmanaged.

---

## Known Limitations Summary

The vehicle may stall due to:
- Misalignment between waypoints and drivable lane
- Lack of lane curvature estimation
- No arbitration between lane and waypoint control
- Absence of recovery logic

---

## Planned Next Steps

Short-term:
- Lane centerline and curvature estimation
- Blended lane/waypoint steering
- Stuck detection and recovery state

Mid-term:
- Lane-aware path planning
- Road-constrained waypoint generation
- Improved supervisor arbitration

---

## Notes

- Model prioritizes clarity over performance
- Diagnostics impact timing
- Parameters are under active tuning
- README will evolve with the project

---

## Summary

This model represents an early but successful integration milestone.
The system switches modes, moves intentionally, and exposes clear paths for improvement.
