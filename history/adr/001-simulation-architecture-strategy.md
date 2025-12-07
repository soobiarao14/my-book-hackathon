# ADR-001: Simulation Architecture Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-05
- **Feature:** 001-humanoid-robotics-capstone
- **Context:** Physical AI capstone project requiring digital twin simulation for safe development, testing, and perception training. Team needs to balance physics fidelity, ROS 2 integration, compute resources, and 12-week timeline constraints.

## Decision

**Hybrid Simulation Architecture**: Gazebo Classic 11 (primary) + NVIDIA Isaac Sim (secondary)

**Gazebo Classic 11** for:
- Primary development and testing environment
- Physics simulation with ODE/Bullet engines
- Full ROS 2 integration via `ros_gz_bridge`
- Student task iteration and debugging
- Final demo execution

**NVIDIA Isaac Sim** for:
- Synthetic perception dataset generation with domain randomization
- Navigation reinforcement learning training
- Reality gap reduction testing (sensor noise, lighting variations)
- GPU-accelerated perception pipeline validation

## Consequences

### Positive

- **Native ROS 2 Integration**: `ros_gz_bridge` provides seamless topic/service bridging without custom middleware development
- **Physics Fidelity**: ODE/Bullet engines accurately model friction, contact dynamics, joint constraints for manipulation tasks
- **Sensor Realism**: Camera, depth, IMU, and joint state plugins match RealSense D435i characteristics
- **Compute Efficiency**: Gazebo runs on RTX 4070 Ti workstation without ray-tracing overhead, leaving GPU resources for Isaac/LLM inference
- **Educational Ecosystem**: Extensive ROS 2 + Gazebo tutorials align with 12-13 week capstone timeline
- **URDF Native**: Direct import of humanoid URDF models without conversion layers
- **Hybrid Benefits**: Leverage Isaac Sim's photorealistic rendering for perception training while maintaining Gazebo's workflow efficiency

### Negative

- **Dual Toolchain Complexity**: Students must learn both Gazebo and Isaac Sim workflows (estimated 4-6 hours additional setup time)
- **Sim-to-Real Gap**: Gazebo physics may not capture all contact dynamics (e.g., soft gripper compliance), requiring physical validation
- **Isaac Sim Licensing**: Requires NVIDIA Omniverse account and compatible GPU (RTX 2000+ series)
- **Version Lock-in**: Gazebo Classic 11 is in maintenance mode (Gazebo Harmonic is successor but has breaking changes)

## Alternatives Considered

### Alternative Stack A: Unity + Robotics Hub (standalone)
- **Components**: Unity 2022+ with Unity Robotics Hub, HDRP rendering
- **Pros**: Photorealistic rendering, cross-platform builds, large asset ecosystem
- **Cons**:
  - Custom TCP bridge required for ROS 2 integration (more complex than `ros_gz_bridge`)
  - HDRP rendering demands high GPU resources (conflicts with LLM inference)
  - Less mature physics for contact-rich manipulation compared to Gazebo
  - Steeper learning curve for students without Unity experience
- **Why Rejected**: ROS 2 integration complexity and compute resource contention outweigh rendering benefits

### Alternative Stack B: Isaac Sim Standalone (primary)
- **Components**: Isaac Sim 2024.1 with native ROS 2 bridge, PhysX 5 engine
- **Pros**:
  - Best-in-class physics (PhysX 5 with GPU acceleration)
  - RTX ray-tracing for photorealistic sensor simulation
  - Integrated Isaac ROS pipelines (VSLAM, object detection)
- **Cons**:
  - Very high GPU usage (8-12GB VRAM continuous) conflicts with LLM inference
  - Slower iteration speed than Gazebo (heavier simulation environment)
  - NVIDIA-centric ecosystem (less transferable knowledge)
  - Steeper learning curve (Omniverse Kit framework)
- **Why Rejected**: GPU resource contention and iteration speed don't align with 12-week capstone constraints

### Alternative Stack C: Simulation-only (no physical deployment)
- **Components**: Gazebo Classic 11 only, no Jetson Orin deployment
- **Pros**: Fastest development cycle, no hardware debugging complexity
- **Cons**:
  - Fails to meet SC-006 success criteria (sim-to-real transfer <20% gap)
  - Misses educational goal of physical AI deployment
  - No validation of hardware constraints (battery management, E-stop)
- **Why Rejected**: Does not satisfy project requirements for sim-to-real validation

## References

- Feature Spec: `specs/001-humanoid-robotics-capstone/spec.md`
- Implementation Plan: `specs/001-humanoid-robotics-capstone/plan.md`
- Research Document: `specs/001-humanoid-robotics-capstone/research.md` (Decision 1)
- Related ADRs: ADR-002 (Distributed Compute), ADR-003 (ROS 2 Architecture)
- Evaluator Evidence: `history/prompts/001-humanoid-robotics-capstone/003-implementation-planning.plan.prompt.md`
