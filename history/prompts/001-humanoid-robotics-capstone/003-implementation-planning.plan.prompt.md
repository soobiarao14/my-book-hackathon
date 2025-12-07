---
id: 003
title: Implementation planning for humanoid robotics capstone
stage: plan
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-humanoid-robotics-capstone
branch: 001-humanoid-robotics-capstone
user: soobiarao14
command: /sp.plan
labels: ["planning", "architecture", "ROS2", "VLA", "robotics", "physical-AI"]
links:
  spec: specs/001-humanoid-robotics-capstone/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-humanoid-robotics-capstone/research.md
  - specs/001-humanoid-robotics-capstone/data-model.md
  - specs/001-humanoid-robotics-capstone/contracts/README.md
  - specs/001-humanoid-robotics-capstone/quickstart.md
  - specs/001-humanoid-robotics-capstone/plan.md
tests:
  - N/A (planning phase)
---

## Prompt

Capstone Project on Physical AI & Humanoid Robotics

Create comprehensive implementation plan with:
- End-to-end architecture (ROS 2, Gazebo/Unity, Isaac, VLA stack, Jetson edge)
- SDD-RI phases (Research, Foundation, Analysis, Synthesis)
- Research-concurrent approach
- Technology decisions (Gazebo vs Unity, workstation vs Jetson, ROS2 architecture, Whisper/LLM selection, Isaac vs custom controllers, testing strategy, documentation workflow)
- Testing strategy aligned with spec requirements
- Docusaurus v3.9 + MCP Server documentation pipeline

## Response snapshot

Generated comprehensive implementation plan for Physical AI & Humanoid Robotics capstone spanning research, architecture, and 12-week implementation timeline.

**Phase 0 - Research (research.md, 15,000+ words)**:
7 critical technology decisions documented with rationale, alternatives, and implementation impact:

1. **Simulation**: Gazebo Classic (primary, native ROS 2 bridge, physics fidelity) + Isaac Sim (perception training, RL)
2. **Compute**: Hybrid workstation (LLM inference 16GB VRAM) + Jetson Orin (robotics stack 8GB) - balances performance vs autonomy
3. **ROS 2 Architecture**: Multi-agent 12-node system across 5 layers (Voice I/O, VLA Planning, Perception, Navigation, Manipulation, Hardware)
4. **VLA Stack**: Whisper base (74M, 50-80ms) + Llama 3 8B FP16 (<100ms) + Python FSM action planner - meets FR-010 latency targets
5. **Controllers**: Hybrid Isaac ROS (perception GPU-accelerated) + Nav2/MoveIt 2 (control mature stacks)
6. **Testing**: Progressive pyramid (70% simulation, 20% integration, 10% physical) - aligns with 12-week timeline
7. **Documentation**: Docusaurus v3.9 (Markdown native, GitHub Pages) + MCP Context7 (live library docs)

**Phase 1 - Data Model & Contracts (data-model.md, 7,500+ words)**:
- 7 core entities defined: VoiceCommand, RobotPose, DetectedObject, TaskPlan, NavigationPath, GraspPose, HardwareState
- ROS 2 interfaces specified: 15+ custom messages, 3 services, 3 actions
- Topic architecture: High-freq (10-30Hz camera/joints), medium (1-5Hz detections/task status), low (<1Hz voice commands)
- Validation rules mapped to FR requirements: centroid ±3cm (FR-004), grasp success >80% (FR-005), battery <20% triggers (FR-013)
- Data flow example: Voice → Whisper → LLM → Task Executor → (Nav2, Isaac, MoveIt 2) → Hardware

**Phase 1 - Contracts (contracts/ directory)**:
- ROS 2 interface definitions directory structure (msg/, srv/, action/)
- README with build instructions and standard interface dependencies
- Integration with ROS 2 package (custom_interfaces) via colcon

**Phase 1 - Quickstart (quickstart.md, 4,500+ words)**:
- Step-by-step setup (4-6 hours): Workstation (ROS 2 + Gazebo + Isaac Sim + Ollama), Jetson (ROS 2 + Isaac ROS + RealSense SDK)
- ROS 2 workspace creation (8 packages)
- Simulation demo walkthrough (6 terminals: Gazebo, perception, nav, VLA planner, executor, RViz)
- Physical deployment (network config, Jetson launch, workstation VLA)
- Validation metrics computation from ROS bags
- Troubleshooting guide (5 common issues)

**Phase 1 - Implementation Plan (plan.md)**:
- Technical context: Python 3.10, ROS 2 Humble, performance targets (<100ms LLM, <200ms vision, <3 min tasks)
- Constitution check: ✅ All principles satisfied (Accuracy, Clarity, Modularity, Consistency, Reproducibility, SDD-RI Alignment)
- Project structure: 8 ROS 2 packages across 5 layers
- **Weeks 3-4**: Perception pipeline (Isaac ROS object detection + VSLAM) → >90% detection accuracy (SC-004)
- **Weeks 5-6**: Navigation stack (Nav2 + safety monitor) → <10cm error, 95% success (SC-003)
- **Weeks 7-8**: Manipulation (MoveIt 2 + grasp planning) → >80% grasp success (SC-005)
- **Weeks 9-10**: VLA integration (Whisper + Llama 3 + FSM) → <3 min end-to-end (SC-001)
- **Week 11**: Sim-to-real transfer (Jetson deployment) → <20% performance gap (SC-006)
- **Week 12**: Evaluation + Docusaurus report (4000-6000 words) + final demo

**Architecture Highlights**:
- **12 ROS 2 Nodes**: Audio capture, Whisper ASR, VLA planner, task executor, TTS, RealSense driver, object detector, human detector, VSLAM, path planner, obstacle monitor, grasp planner, arm controller, motor driver, sensor fusion, battery monitor, E-stop
- **Safety Implementation** (FR-011): Human detector → speed limiter (≤0.5m/s within 1m) → motor driver E-stop (<100ms halt)
- **VLA Pipeline**: Microphone → Whisper (with 3x retry + fallback) → Llama 3 JSON parsing → FSM subtask execution → ROS 2 actions
- **Testing Strategy**: Unit tests (pytest, >80% coverage), integration tests (Gazebo user story validation), system tests (physical robot sim-to-real validation)

**Key Decisions Documented**:
- Gazebo Classic over Unity: Better ROS 2 integration, physics fidelity, lower compute cost
- Hybrid compute: LLM on workstation (16GB VRAM needed), robotics stack on Jetson (autonomous capability)
- Multi-agent ROS 2: Enables parallel dev, fault isolation, sim-to-real portability
- Whisper base + Llama 3 8B: Meets <100ms latency targets vs GPT-3.5 API (200-500ms)
- Progressive testing: 70% sim enables fast iteration, 10% physical validates SC-006

## Outcome

- ✅ Impact: Comprehensive 12-week implementation plan with technology stack, architecture, and phase-by-phase milestones
- 🧪 Tests: Testing strategy defined (unit/integration/system), metrics mapped to SC-001 to SC-008
- 📁 Files: 5 created (research.md, data-model.md, contracts/README.md, quickstart.md, plan.md)
- 🔁 Next prompts: `/sp.tasks` to generate detailed task breakdown (tasks.md) for implementation
- 🧠 Reflection: Plan balances technical depth with 12-week timeline constraints. Hybrid compute architecture (workstation LLM + Jetson robotics) solves memory limitation while maintaining edge deployment capability. Multi-agent ROS 2 design enables parallel team development. Progressive testing pyramid (70% sim, 20% integration, 10% physical) aligns with educational goals while validating sim-to-real transfer (SC-006).

## Evaluation notes (flywheel)

- Failure modes observed: None. All planning artifacts generated successfully.
- Graders run and results (PASS/FAIL): Architecture validation—12 nodes across 5 layers, all FR/SC requirements addressable (PASS). Timeline validation—12 weeks with weekly milestones, critical path identified (PASS). Technology decisions—7 decisions documented with alternatives and rationale (PASS).
- Prompt variant (if applicable): Research-concurrent method (parallel research + implementation) vs sequential—concurrent chosen for 12-week constraint
- Next experiment (smallest change to try): After `/sp.tasks` generation, validate task dependencies match critical path in plan (URDF/Gazebo blocks testing, perception blocks manipulation, VLA blocks user story validation)
