# Module 2: The Digital Twin (Gazebo & Unity)

**Duration:** Weeks 6-7 | Chapters 6-7
**Focus:** Physics Simulation and Environment Building

---

## Module Overview

Create photorealistic digital twins for testing humanoid robots before physical deployment. This module covers both physics-accurate simulation with Gazebo and high-fidelity rendering with Unity.

By the end of this module, you'll be able to simulate complete robotic systems with sensors (cameras, LIDAR, IMU) in realistic environments, bridging the gap between virtual prototyping and real-world deployment.

---

## Learning Objectives

By completing Module 2, you will be able to:

✅ **Simulate Physics** - Create accurate physics simulations with Gazebo
✅ **Model Environments** - Build SDF worlds with obstacles and terrains
✅ **Integrate Sensors** - Simulate cameras, LIDAR, and IMU sensors
✅ **Render Photorealistically** - Use Unity for high-fidelity visuals
✅ **Bridge to ROS 2** - Connect simulators to ROS 2 via ros_gz_bridge

---

## Chapter Breakdown

### Chapter 6: Robot Simulation with Gazebo (Week 6)
Master physics simulation with Gazebo Classic and Fortress.

**Topics:**
- Gazebo architecture and physics engines (ODE, Bullet, Dart)
- SDF (Simulation Description Format)
- Sensor plugins: camera, depth camera, LIDAR, IMU
- Contact sensors and force-torque sensors
- ros_gz_bridge: Bidirectional ROS 2 integration
- World building: terrains, obstacles, lighting

**Project:** Humanoid obstacle course with complete sensor suite

---

### Chapter 7: High-Fidelity Rendering with Unity (Week 7)
Use Unity for photorealistic rendering and human-robot interaction simulation.

**Topics:**
- Unity Robotics Hub setup and configuration
- URDF Importer for Unity
- Photorealistic environments with HDRP
- Simulating human-robot interaction scenarios
- RGB-D camera simulation with perception data
- TCP/IP ROS 2 bridge (ROS-TCP-Connector)
- Articulation Body for physics

**Project:** Living room environment with RGB-D simulation for object detection

---

## Why Digital Twins?

**Test Before You Build:**
- Validate algorithms in simulation before deploying to expensive hardware
- Iterate quickly without physical robot downtime
- Test dangerous scenarios safely (falling, collisions)

**Generate Training Data:**
- Synthetic datasets for computer vision models
- Domain randomization for sim-to-real transfer
- Perfect ground truth labels (depth, segmentation, poses)

**Parallel Development:**
- Software team works in simulation while hardware team builds robot
- Multiple engineers test in parallel environments
- CI/CD pipelines run automated tests in simulation

---

## Module Assessment

**Gazebo Simulation Implementation**

Create a complete digital twin system that demonstrates:
1. Humanoid robot model in Gazebo with URDF
2. Custom environment with obstacles and terrain
3. Simulated sensors: RGB-D camera, LIDAR, IMU
4. ROS 2 integration publishing sensor data
5. Navigation task: robot navigates obstacle course

**Deliverables:**
- Gazebo world file (.sdf)
- Launch file to start simulation and ROS 2 nodes
- Video demonstration of robot navigating environment
- Documentation explaining sensor configurations

---

## Prerequisites

- **Module 1 Complete:** ROS 2 fundamentals, URDF modeling
- **3D Basics:** Understanding of 3D coordinates, rotations (quaternions)
- **Hardware:**
  - Minimum: 8GB RAM, integrated graphics
  - Recommended: 16GB+ RAM, dedicated GPU (GTX 1660 or better)

---

## Technical Stack

**Gazebo:** Gazebo Classic 11 (stable with ROS 2 Humble)
**Unity:** Unity 2022.3 LTS
**Physics Engines:** ODE (default), Bullet, Dart
**ROS 2 Bridges:**
- `ros_gz_bridge` (Gazebo ↔ ROS 2)
- `ROS-TCP-Connector` (Unity ↔ ROS 2)

**Sensors Simulated:**
- RGB Cameras (camera plugin)
- Depth Cameras (depth_camera plugin)
- LIDAR (ray/gpu_ray plugin)
- IMU (imu plugin)
- Contact sensors (contact plugin)

---

## Start Learning

Ready to begin? Start with:

→ [Chapter 6: Robot Simulation with Gazebo](ch06-gazebo-simulation.md)

Or jump ahead:
- [Chapter 7: High-Fidelity Rendering with Unity](ch07-unity-rendering.md)

---

**Previous Module:** [Module 1: The Robotic Nervous System](../module-1-ros2/index.md)
**Next Module:** [Module 3: The AI-Robot Brain (NVIDIA Isaac)](../module-3-isaac/index.md)
