# Humanoid Robotics Capstone Project

**12-week capstone project implementing voice-commanded object retrieval using ROS 2, Vision-Language-Action (VLA) pipeline, and physical humanoid robot.**

## Project Overview

This project implements a complete robotics system that:
- Accepts voice commands from users ("Bring me the red cup from the kitchen")
- Parses commands using an LLM (Llama 3 8B) into structured tasks
- Perceives the environment using RGB-D cameras and object detection
- Navigates autonomously using Visual SLAM and Nav2
- Manipulates objects using grasp planning and MoveIt 2
- Provides voice feedback and status updates

**Key Features:**
- Vision-Language-Action (VLA) pipeline with Whisper ASR + Llama 3 8B
- Real-time object detection and 3D localization (±3cm accuracy)
- Autonomous navigation with obstacle avoidance
- Multi-step task execution with progress feedback
- Safety systems (E-stop, human detection, battery monitoring)
- Hybrid compute architecture (Workstation GPU + Jetson Orin NX)

## Technology Stack

**Core Framework:**
- ROS 2 Humble (Ubuntu 22.04 LTS)
- Python 3.10+
- Gazebo Classic 11 (simulation)

**AI/ML:**
- OpenAI Whisper (ASR via API)
- Llama 3 8B (LLM via Groq API)
- YOLOv8 + TensorRT (object detection)
- NVIDIA Isaac ROS (GPU-accelerated perception)

**Robotics:**
- Nav2 (navigation stack)
- MoveIt 2 (manipulation planning)
- Visual SLAM (Isaac ROS or ORB-SLAM3)

**Hardware:**
- Intel RealSense D435i/D455 (RGB-D camera)
- NVIDIA Jetson Orin NX 8GB (robot controller)
- Workstation with RTX 4070 Ti (LLM inference)

## Project Structure

```
.
├── ros2_ws/                      # ROS 2 workspace
│   ├── src/
│   │   ├── custom_interfaces/    # Custom ROS 2 messages/services/actions
│   │   ├── vla_planning/         # Voice-Language-Action planning
│   │   ├── perception/           # Camera drivers and object detection
│   │   ├── navigation/           # SLAM and path planning
│   │   ├── manipulation/         # Grasp planning and arm control
│   │   └── hw_interface/         # Motor drivers, sensors, safety
│   ├── launch/                   # System-level launch files
│   └── config/                   # Configuration files
├── gazebo_ws/                    # Gazebo simulation assets
│   ├── models/                   # Robot URDF and object models
│   ├── worlds/                   # Simulation environments
│   └── launch/                   # Gazebo launch files
├── specs/                        # Design specifications
│   └── 001-humanoid-robotics-capstone/
│       ├── spec.md               # Feature requirements
│       ├── plan.md               # Architecture and design
│       ├── tasks.md              # Task breakdown (101 tasks)
│       ├── data-model.md         # Data structures
│       └── contracts/            # API contracts
├── .specify/                     # Spec-Kit Plus templates
└── README.md                     # This file
```

## Quick Start

### Prerequisites

**Ubuntu 22.04 LTS** (required for ROS 2 Humble native support)

**Install ROS 2 Humble:**
```bash
# TODO: Add ROS 2 installation instructions
# Follow: https://docs.ros.org/en/humble/Installation.html
```

**Install Dependencies:**
```bash
# TODO: Create requirements.txt and install script
# Will include: pyrealsense2, opencv-python, pytorch, tensorrt, etc.
```

**Set up API Keys:**
```bash
# Create .env file with API keys
cp .env.example .env
# Edit .env and add:
# OPENAI_API_KEY=your_key_here
# GROQ_API_KEY=your_key_here
```

### Build Workspace

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Run Simulation

```bash
# Launch full system in Gazebo
ros2 launch simulation.launch.py
```

### Run on Physical Robot

**On Workstation (runs LLM and perception):**
```bash
export ROS_DOMAIN_ID=42  # Match Jetson
ros2 launch full_system_workstation.launch.py
```

**On Jetson Orin NX (runs navigation and control):**
```bash
export ROS_DOMAIN_ID=42  # Match workstation
ros2 launch full_system_jetson.launch.py
```

## System Architecture

### Data Flow

```
User Voice Command
  ↓ (microphone)
Audio Capture Node
  ↓ (WAV file)
Whisper ASR Node
  ↓ (transcript)
VLA Planner Node (LLM)
  ↓ (TaskPlan)
Task Executor Node
  ├→ Object Detection → Perception
  ├→ Path Planning → Navigation
  └→ Grasp Execution → Manipulation
       ↓ (success/failure)
TTS Feedback → User
```

### ROS 2 Package Overview

**custom_interfaces** - Custom message/service/action definitions
- `VoiceCommand.msg` - Voice command with parsed fields
- `TaskPlan.msg` - Multi-step task plan
- `DetectObjects.srv` - Object detection service
- `PickObject.action` - Pick-and-place action

**vla_planning** - Voice-Language-Action pipeline
- `audio_capture_node` - Voice recording with VAD
- `whisper_asr_node` - Speech-to-text transcription
- `vla_planner_node` - LLM-based action parsing
- `task_executor_node` - FSM-based task coordination
- `tts_node` - Text-to-speech feedback

**perception** - Vision system
- `realsense_node` - RGB-D camera driver
- `object_detector_node` - 3D object detection
- `human_detector_node` - Human detection for safety

**navigation** - Autonomous navigation
- `vslam_node` - Visual SLAM localization
- `path_planner_node` - Nav2 path planning
- `obstacle_monitor_node` - Real-time obstacle detection

**manipulation** - Arm control
- `grasp_planner_node` - Grasp pose generation
- `arm_controller_node` - MoveIt 2 motion execution

**hw_interface** - Hardware abstraction
- `motor_driver_node` - Wheel and arm motor control
- `sensor_fusion_node` - Multi-sensor odometry
- `battery_monitor_node` - Battery monitoring
- `estop_node` - Emergency stop safety

## Development Workflow

This project follows **Spec-Driven Development (SDD-RI)** methodology:

1. **Specification** (`specs/001-humanoid-robotics-capstone/spec.md`)
   - Functional requirements (FR-001 to FR-020)
   - Success criteria with measurable metrics
   - User stories with acceptance tests

2. **Planning** (`specs/001-humanoid-robotics-capstone/plan.md`)
   - Architecture decisions
   - Tech stack justification
   - Phase breakdown (0-6)

3. **Tasks** (`specs/001-humanoid-robotics-capstone/tasks.md`)
   - 101 actionable tasks
   - Dependency ordering
   - Acceptance criteria per task

4. **Implementation** (this scaffold)
   - ROS 2 packages with stub nodes
   - Launch files and configuration
   - TODO comments for team implementation

## Testing Strategy

**Unit Tests** (per package):
```bash
colcon test --packages-select vla_planning
colcon test-result --verbose
```

**Integration Tests**:
- Perception: Object detection accuracy, latency
- Navigation: Path planning success rate, position error
- Manipulation: Grasp success rate >70%
- End-to-end: Full task completion rate

**Simulation Tests**:
- All tests run in Gazebo before physical deployment
- Success criteria from spec.md must pass

## Safety Features

1. **Emergency Stop (E-stop)**
   - Physical button halts all motion within 100ms
   - Requires manual reset to resume

2. **Human Detection**
   - Speed limited to 0.5 m/s when human within 1m
   - Automatic stop if human enters danger zone

3. **Obstacle Avoidance**
   - Emergency stop if obstacle <0.5m ahead
   - Real-time monitoring at 20Hz

4. **Battery Monitoring**
   - Task abort at <15% battery
   - Critical shutdown at <5% battery

## Performance Requirements

| Subsystem | Metric | Target | Success Criteria |
|-----------|--------|--------|------------------|
| ASR | Latency | <2s | FR-003, SC-002 |
| LLM | Latency | <100ms | FR-005, SC-002 |
| Object Detection | Accuracy | ±3cm @ 2m | FR-004, SC-004 |
| Object Detection | Detection Rate | >90% uncluttered | SC-004 |
| Navigation | Position Error | <10cm | FR-009, SC-006 |
| Navigation | Heading Error | <5° | SC-006 |
| Manipulation | Grasp Success | >70% | FR-016, SC-009 |
| E-stop | Response Time | <100ms | FR-012, SC-010 |

## Hardware Setup

**Workstation (LLM host):**
- NVIDIA RTX 4070 Ti (LLM inference, Isaac ROS)
- Ubuntu 22.04 LTS
- ROS 2 Humble

**Jetson Orin NX (robot controller):**
- 8GB unified memory
- Ubuntu 22.04 (JetPack)
- ROS 2 Humble

**Sensors:**
- Intel RealSense D435i/D455 (RGB-D)
- IMU (optional, improves SLAM)

**Actuators:**
- Differential drive mobile base
- 5-7 DOF robotic arm
- Parallel jaw gripper (8cm opening)

## Documentation

Full documentation available in:
- `specs/001-humanoid-robotics-capstone/spec.md` - Requirements
- `specs/001-humanoid-robotics-capstone/plan.md` - Architecture
- `specs/001-humanoid-robotics-capstone/tasks.md` - Task breakdown
- `specs/001-humanoid-robotics-capstone/data-model.md` - Data structures
- `specs/001-humanoid-robotics-capstone/contracts/` - API contracts

## Contributing

This is a 12-week capstone project. Follow the SDD-RI workflow:
1. Read spec.md for requirements
2. Follow plan.md architecture
3. Implement tasks from tasks.md in order
4. Mark tasks complete with `[X]` in tasks.md
5. Test thoroughly in simulation first

## License

MIT License (update as needed for your institution)

## Contact

Capstone Team - capstone@example.com

## Acknowledgments

- NVIDIA Isaac ROS for GPU-accelerated perception
- Open Robotics for ROS 2 and Gazebo
- OpenAI for Whisper ASR
- Groq for fast Llama 3 inference
- Meta for Llama 3 model
