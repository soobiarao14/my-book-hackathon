# ADR-002: Distributed Compute Architecture

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-05
- **Feature:** 001-humanoid-robotics-capstone
- **Context:** VLA pipeline requires Llama 3 8B (16GB VRAM FP16) but Jetson Orin NX/Nano has only 8GB unified memory. Need to balance development speed, autonomous deployment capability, performance targets (<100ms LLM latency), and cost constraints for capstone project.

## Decision

**Hybrid Compute Architecture**: Workstation offload + Jetson Orin edge execution

### Workstation (NVIDIA RTX 4070 Ti + 64GB RAM)
- LLM inference (Llama 3 8B FP16) via Ollama or vLLM
- Whisper ASR (base model, 74M parameters)
- Gazebo simulation hosting
- Development environment (ROS 2 Humble, Isaac SDK, RViz, rqt)
- VLA planner node (`/vla_planner_node`)

### Jetson Orin Nano/NX (8GB Unified Memory)
- All non-LLM ROS 2 nodes (perception, navigation, manipulation, hardware interface)
- Isaac ROS perception pipelines (object detection, VSLAM) with TensorRT acceleration
- Nav2 navigation stack (path planning, costmap processing)
- MoveIt 2 manipulation planning (OMPL, collision checking)
- Motor control and sensor I/O drivers
- Real-time safety monitors (E-stop, human detection, battery management)

### Network Configuration
- Development: Workstation ↔ Jetson over WiFi/Ethernet (lab network)
- ROS 2 DDS discovery via `ROS_DOMAIN_ID` (shared across machines)
- Fallback: Deploy smaller LLM (Llama 3.2 3B INT8 quantized) to Jetson if network unavailable for final demo

## Consequences

### Positive

- **Performance**: LLM runs on workstation with 16GB VRAM (Llama 3 8B FP16) achieving <100ms inference target, while Jetson 8GB is insufficient
- **Deployability**: Core robotics stack (perception, navigation, control) runs autonomously on Jetson after development phase
- **Development Speed**: Students iterate on workstation with full debugging tools (RViz, rqt, Gazebo) while Jetson handles real sensors
- **Cost Efficiency**: Single shared workstation ($2000) vs. multiple AGX Orin 64GB units ($1999 each) for 4-5 team members
- **Parallel Development**: Team members can develop/test nodes on workstation while Jetson is occupied by another member
- **Graceful Degradation**: Network failure mode supported (fallback to keyboard input per FR-001/FR-002)

### Negative

- **Network Dependency**: LLM inference requires workstation reachability (WiFi/Ethernet), adding latency (20-50ms network overhead)
- **Deployment Complexity**: Students must manage two-machine ROS 2 configuration (DDS discovery, topic remapping)
- **Not Fully Autonomous**: Final robot cannot operate independently without workstation connection during voice-commanded tasks
- **Debugging Complexity**: Distributed system issues (network drops, clock sync) harder to diagnose than single-machine setup

## Alternatives Considered

### Alternative Stack A: Workstation-Only (No Jetson)
- **Components**: All ROS 2 nodes + simulation on workstation, tethered physical robot
- **Pros**:
  - Simplest setup (single machine configuration)
  - Best performance (RTX 4070 Ti for all workloads)
  - Full debugging access (GDB, valgrind, profilers)
- **Cons**:
  - Robot permanently tethered via USB/Ethernet (fails autonomous deployment goal)
  - Does not validate edge deployment constraints (Jetson performance, thermal limits)
  - Misses educational value of embedded systems deployment
- **Why Rejected**: Fails to meet SC-006 (sim-to-real transfer) and educational goals for edge AI deployment

### Alternative Stack B: Jetson-Only (8GB Unified Memory)
- **Components**: All nodes including LLM on Jetson Orin NX/Nano
- **Pros**:
  - Fully autonomous operation (no network dependency)
  - Realistic embedded constraints (memory, compute, power budget)
  - Best sim-to-real fidelity
- **Cons**:
  - 8GB insufficient for Llama 3 8B FP16 (requires 16GB VRAM)
  - INT8 quantization (Llama 3 8B) fits but accuracy drops below 85% (violates SC-002)
  - Llama 3.2 3B fits but parsing accuracy for multi-step tasks drops to 78-82%
  - Slow development iteration (RViz on Jetson is sluggish)
- **Why Rejected**: Cannot meet <100ms LLM latency (FR-010) and >85% intent accuracy (SC-002) with 8GB memory constraints

### Alternative Stack C: Cloud Offload (AWS Lambda + API Gateway)
- **Components**: ROS 2 on Jetson, LLM hosted on AWS with API calls
- **Pros**:
  - Infinite LLM scalability (larger models if needed)
  - No workstation hardware requirement
  - Pay-per-use cost model
- **Cons**:
  - API latency 200-500ms (violates FR-010 <100ms target)
  - Internet dependency (fails in offline lab environments)
  - Ongoing API costs ($0.002 per request × 100 tests = $0.20, but scales with usage)
  - Less educational value (abstracts away inference details)
- **Why Rejected**: Latency constraints and network reliability requirements not met

### Alternative Stack D: AGX Orin 64GB (All-in-One)
- **Components**: Jetson AGX Orin 64GB runs all nodes including LLM
- **Pros**:
  - Fully autonomous (no network dependency)
  - Sufficient memory for Llama 3 8B FP16
  - Single-machine simplicity
- **Cons**:
  - Cost: $1999 per unit × 4-5 teams = $8000-10000
  - Still slower iteration than workstation (no RViz acceleration)
  - Thermal throttling under combined LLM + perception load
- **Why Rejected**: Budget constraints prohibitive for capstone project scale

## References

- Feature Spec: `specs/001-humanoid-robotics-capstone/spec.md` (FR-010 latency requirement)
- Implementation Plan: `specs/001-humanoid-robotics-capstone/plan.md`
- Research Document: `specs/001-humanoid-robotics-capstone/research.md` (Decision 2)
- Related ADRs: ADR-001 (Simulation), ADR-003 (ROS 2 Architecture), ADR-004 (VLA Stack)
- Evaluator Evidence: `history/prompts/001-humanoid-robotics-capstone/003-implementation-planning.plan.prompt.md`
