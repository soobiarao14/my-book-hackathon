# ADR-003: ROS 2 Multi-Agent System Design

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-05
- **Feature:** 001-humanoid-robotics-capstone
- **Context:** Humanoid robotics system requires coordination of voice I/O, VLA planning, perception, navigation, and manipulation subsystems. Need to balance modularity (parallel team development), testability, fault isolation, and complexity for 12-week capstone timeline.

## Decision

**Multi-Agent Modular Architecture** with centralized task planner

### Node Structure (12 Primary Nodes Across 5 Layers)

#### 1. Voice I/O Layer
- `/audio_capture_node`: Microphone array input, Voice Activity Detection (VAD)
- `/whisper_asr_node`: Speech-to-text (Whisper base model)
- `/tts_node`: Text-to-speech feedback (pyttsx3/Festival)

#### 2. VLA Planning Layer (Workstation)
- `/vla_planner_node`: LLM command parsing + action sequence planning
- `/task_executor_node`: Subtask sequencing, state machine, failure handling

#### 3. Perception Layer (Jetson + Isaac ROS)
- `/realsense_node`: RGB-D camera driver (Intel RealSense SDK)
- `/object_detector_node`: YOLO/Isaac DNN object detection + 3D localization
- `/human_detector_node`: Person detection for safety compliance (FR-011)

#### 4. Navigation Layer (Jetson + Nav2)
- `/vslam_node`: Visual SLAM (Isaac Visual SLAM or RTAB-Map)
- `/path_planner_node`: Global/local path planning (Nav2 stack)
- `/obstacle_monitor_node`: Dynamic obstacle avoidance + safety speed limits

#### 5. Manipulation Layer (Jetson + MoveIt 2)
- `/grasp_planner_node`: Grasp pose generation from object bounding box
- `/arm_controller_node`: Joint trajectory execution + force sensing

#### 6. Hardware Interface Layer (Jetson)
- `/motor_driver_node`: Actuator commands (CAN bus or serial)
- `/sensor_fusion_node`: IMU + joint encoders fusion
- `/battery_monitor_node`: Battery level publishing (FR-013)
- `/estop_node`: Emergency stop monitoring (FR-011)

### Communication Patterns

**Topics** (high-frequency data):
- Camera feeds: `/camera/rgb`, `/camera/depth` (10-30 Hz)
- Robot state: `/joint_states`, `/odom` (10-30 Hz)
- Safety: `/battery_level`, `/estop_status` (1-5 Hz)

**Services** (request-response):
- `/detect_objects`: Synchronous object detection
- `/plan_path`: Path planning request
- `/compute_grasp_pose`: Grasp planning request

**Actions** (long-running goals with feedback):
- `/navigate_to_pose`: Navigation with progress feedback
- `/execute_trajectory`: Arm trajectory with force feedback
- `/pick_object`: Multi-step grasp action

### Centralized Task Planner Rationale
- Multi-step tasks (FR-006) require global state tracking
- Single `/task_executor_node` prevents race conditions between subtasks
- Aligns with VLA pipeline: one LLM call → one task plan → sequential subtask execution

## Consequences

### Positive

- **Modularity**: Each node is independently testable (unit tests per node), enabling parallel development
- **Fault Isolation**: Perception failure doesn't crash navigation; nodes can restart independently
- **Parallel Development**: Team members work on separate nodes concurrently (5 packages across 4-5 developers)
- **Sim-to-Real Transfer**: Same nodes run in Gazebo and on physical robot (only `/motor_driver_node` implementation changes)
- **Debugging Granularity**: ROS 2 tools (rqt_graph, topic echo, service call) enable fine-grained system inspection
- **Scalability**: ROS 2 namespace separation (`/robot1/...`) enables multi-robot future extension
- **Educational Value**: Students learn industry-standard ROS 2 patterns (publishers, subscribers, services, actions)

### Negative

- **Integration Complexity**: 12 nodes require careful launch file orchestration and parameter management
- **Latency Overhead**: Inter-node communication adds 5-20ms per hop (topic publish → subscribe → process → publish)
- **Debugging Distributed Systems**: Race conditions, message loss, and timing issues harder to diagnose than monolithic code
- **ROS 2 Learning Curve**: Students must understand DDS discovery, QoS policies, lifecycle nodes (estimated 8-12 hours initial learning)

## Alternatives Considered

### Alternative Stack A: Monolithic Single Node
- **Components**: All functionality in one ROS 2 node (or Python script)
- **Pros**:
  - Simplest deployment (single executable)
  - Minimal inter-process communication overhead
  - Easier to debug with traditional tools (debugger, print statements)
- **Cons**:
  - Hard to isolate faults (one bug crashes entire system)
  - Integration tests only (no unit test granularity)
  - Sequential team development (merge conflicts on single codebase)
  - Fragile sim-to-real transfer (tightly coupled to simulation or hardware)
- **Why Rejected**: Does not support parallel team development and lacks fault isolation for 12-week timeline

### Alternative Stack B: Microservices (20+ Fine-Grained Nodes)
- **Components**: Decompose to 20+ nodes (separate nodes for camera driver, image preprocessing, detection inference, 3D projection, etc.)
- **Pros**:
  - Maximum modularity and testability
  - Each service can scale independently
  - Best fault isolation (smallest blast radius)
- **Cons**:
  - Very high complexity (launch files with 20+ nodes)
  - Significant network overhead (10+ message hops for single task)
  - Over-engineering for capstone scope (diminishing returns on modularity)
  - Steeper learning curve for students
- **Why Rejected**: Complexity outweighs benefits for 12-week capstone; optimal granularity is ~12 nodes, not 20+

### Alternative Stack C: Behavior Tree Coordination (No Centralized Planner)
- **Components**: BehaviorTree.CPP library coordinating reactive behaviors (no `/task_executor_node`)
- **Pros**:
  - Reactive to sensor changes (preemptive behaviors)
  - Mature ecosystem (used in Nav2, MoveIt 2)
  - Visual editing tools (Groot)
- **Cons**:
  - Complex multi-step tasks harder to express (7+ subtasks for P2 user story)
  - LLM output must generate behavior tree XML (not simple JSON)
  - Additional learning curve (behavior tree semantics)
  - Less interpretable for debugging than explicit state machine
- **Why Rejected**: Centralized state machine (FSM) in `/task_executor_node` simpler for VLA-driven multi-step tasks

### Alternative Stack D: ROS 1 (Noetic)
- **Components**: ROS 1 Noetic with master node coordination
- **Pros**:
  - More mature ecosystem (more tutorials, packages)
  - Students may have prior ROS 1 experience
- **Cons**:
  - ROS 1 end-of-life in 2025 (outdated for new projects)
  - No native multi-machine support (requires rosbridge for workstation-Jetson split)
  - Worse performance than ROS 2 DDS (centralized master bottleneck)
  - Does not teach current industry standard (ROS 2)
- **Why Rejected**: ROS 2 is industry standard (2024-2025), better supports distributed compute architecture

## References

- Feature Spec: `specs/001-humanoid-robotics-capstone/spec.md` (FR-006 multi-step tasks)
- Implementation Plan: `specs/001-humanoid-robotics-capstone/plan.md`
- Research Document: `specs/001-humanoid-robotics-capstone/research.md` (Decision 3)
- Data Model: `specs/001-humanoid-robotics-capstone/data-model.md` (ROS 2 message definitions)
- Related ADRs: ADR-002 (Distributed Compute), ADR-004 (VLA Stack)
- Evaluator Evidence: `history/prompts/001-humanoid-robotics-capstone/003-implementation-planning.plan.prompt.md`
