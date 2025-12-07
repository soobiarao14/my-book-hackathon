# Module 1: The Robotic Nervous System (ROS 2)

**Duration:** Weeks 1-5 | Chapters 1-5
**Focus:** Middleware for Robot Control

---

## Module Overview

Master ROS 2 (Robot Operating System 2), the middleware that serves as the "nervous system" connecting all robot components. This foundational module teaches you how to build complex robotic systems using publisher-subscriber patterns, services, actions, and parameters.

By the end of this module, you'll understand how to architect multi-node robotic systems, model humanoid robots with URDF, and visualize them in RViz.

---

## Learning Objectives

By completing Module 1, you will be able to:

✅ **Explain Physical AI** - Understand embodied intelligence and its applications
✅ **Design ROS 2 Systems** - Build multi-node architectures with proper communication patterns
✅ **Develop Python Packages** - Create production-ready ROS 2 packages with rclpy
✅ **Configure Launch Files** - Manage complex multi-node systems with parameters
✅ **Model Humanoid Robots** - Use URDF and Xacro to describe robot kinematics

---

## Chapter Breakdown

### Chapter 1: Introduction to Physical AI (Weeks 1-2)
From digital AI to embodied intelligence. Survey the landscape of humanoid robotics.

**Topics:**
- Physical AI vs Traditional AI
- Industry case studies: Tesla Optimus, Figure 01, Boston Dynamics Atlas
- The Physical AI stack: Perception → Reasoning → Action
- Sensor systems: Vision, proprioception, force sensing

**Project:** None (introductory survey)

---

### Chapter 2: ROS 2 Architecture and Core Concepts (Week 3)
Understand nodes, topics, services, and actions. Build your first ROS 2 system.

**Topics:**
- ROS 2 architecture and DDS middleware
- Topics: Publish-subscribe communication
- Services: Request-response patterns
- Actions: Long-running goals with feedback
- Installing ROS 2 Humble on Ubuntu 22.04

**Project:** Simple talker-listener system

---

### Chapter 3: Building ROS 2 Packages with Python (Week 4)
Use rclpy to create production-ready ROS 2 packages.

**Topics:**
- Python vs C++ in ROS 2
- Package structure and colcon build system
- Quality of Service (QoS) profiles
- Parameter management
- Timer callbacks and rate control

**Project:** Sensor data publisher and controller

---

### Chapter 4: Launch Files and Parameter Management (Week 5)
Configure complex multi-node systems with launch files.

**Topics:**
- Python-based launch files
- Namespacing and remapping
- YAML parameter files
- Launch file composition
- Lifecycle management

**Project:** Multi-node humanoid sensor suite

---

### Chapter 5: URDF and Robot Description (Week 5)
Model humanoid robots with URDF and visualize in RViz.

**Topics:**
- URDF format for robot modeling
- Humanoid anatomy: links and joints
- Visual vs collision geometry
- Xacro: XML macros for modularity
- RViz visualization and joint_state_publisher

**Project:** Build and visualize humanoid URDF in RViz

---

## Module Assessment

**ROS 2 Package Development Project**

Build a complete multi-node ROS 2 system that demonstrates:
1. Multiple nodes communicating via topics
2. At least one service for configuration
3. Parameter management via YAML files
4. Launch file to start the entire system
5. URDF model visualized in RViz

**Deliverables:**
- Functional ROS 2 workspace with packages
- Documentation (README with architecture diagram)
- Video demonstration of system running

---

## Prerequisites

- **Python Programming:** Intermediate level (classes, functions, decorators)
- **Linux Command Line:** Basic familiarity with Ubuntu terminal
- **Hardware:** Ubuntu 22.04 machine (native, VM, or WSL2)

---

## Technical Stack

**Operating System:** Ubuntu 22.04 LTS
**ROS Version:** ROS 2 Humble Hawksbill (LTS until 2027)
**Build Tool:** colcon
**Python Version:** Python 3.10+
**Visualization:** RViz2

---

## Start Learning

Ready to begin? Start with:

→ [Chapter 1: Introduction to Physical AI](ch01-intro-physical-ai.md)

Or jump to a specific topic:
- [Chapter 2: ROS 2 Architecture](ch02-ros2-architecture.md)
- [Chapter 3: Building ROS 2 Packages](ch03-ros2-packages.md)
- [Chapter 4: Launch Files](ch04-launch-files.md)
- [Chapter 5: URDF and Robot Description](ch05-urdf-robot-description.md)

---

**Next Module:** [Module 2: The Digital Twin (Gazebo & Unity)](../module-2-digital-twin/index.md)
