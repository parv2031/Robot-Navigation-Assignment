# Robot Navigation — Path Smoothing & Trajectory Tracking

A ROS2-based autonomous navigation system for TurtleBot3 that performs smooth trajectory generation and accurate path tracking inside a Gazebo simulation environment.

The project implements a complete navigation pipeline including:
- Path smoothing
- Time-parameterized trajectory generation
- PID-based trajectory tracking
- RViz visualization of planned vs actual trajectory

Built using **ROS2**, **Gazebo**, **RViz2**, and **C++17**. :contentReference[oaicite:0]{index=0}

---

# Features

- Gradient-based path smoothing
- Linear interpolation for dense waypoint generation
- Time-parameterized trajectory generation
- PD/PID trajectory tracking controller
- Real-time RViz trajectory visualization
- YAML-based configurable waypoints
- Differential drive robot simulation using TurtleBot3
- Modular ROS2 architecture :contentReference[oaicite:1]{index=1}

---

# Tech Stack

- ROS2 (Humble / Iron)
- TurtleBot3 Burger
- Gazebo 11
- RViz2
- C++17 :contentReference[oaicite:2]{index=2}

---

# System Architecture

Waypoints (YAML)
↓
Path Smoother
↓
Trajectory Generator
↓
PID Controller
↓
TurtleBot3 Robot

The system uses ROS2 topics for communication between modules. :contentReference[oaicite:3]{index=3}

---

# ROS2 Topics

| Topic | Description |
|---|---|
| `/planned_path` | Smoothed planned trajectory |
| `/robot_path` | Actual robot trajectory |
| `/cmd_vel` | Velocity commands |
| `/odom` | Robot odometry feedback |
| `/scan` | LiDAR scan data | :contentReference[oaicite:4]{index=4}

---

# Repository Structure

```bash
robot_navigation/
├── src/
│   ├── controller.cpp
│   ├── path_smoother.cpp
│   └── trajectory_generator.cpp
├── include/robot_navigation/
├── launch/
│   ├── sim.launch.py
│   └── navigation.rviz
├── config/
│   └── controller_params.yaml
├── CMakeLists.txt
└── package.xml
