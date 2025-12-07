<!--
SYNC IMPACT REPORT:
Version Change: v1.0.0 (Book Creation) → v2.0.0 (Physical AI & Robotics Engineering)
Rationale: Major domain shift from book authoring to robotics engineering. All principles redefined for Physical AI, humanoid robotics, and ROS 2 development.

Modified Principles:
- REPLACED: Dual-Output Philosophy → Implementation-First Philosophy
- REPLACED: Accuracy (JS/React/Next.js) → Accuracy (Python/ROS 2/Robotics Stack)
- REPLACED: Clarity (Flesch-Kincaid 8-10) → Safety-First Design
- REPLACED: Modularity (Chapters) → Modularity (ROS 2 Packages)
- REPLACED: Consistency (Terminology) → Sim-to-Real Transferability
- KEPT: Reproducibility (adapted for robotics installations)
- REPLACED: Docusaurus Compliance → Observable Systems
- REPLACED: (none) → Performance Constraints (new principle)
- KEPT: SDD-RI Alignment (unchanged)

Added Sections:
- Robotics Project Standards (replaces Book Production Standards)
- ROS 2 Package Requirements
- Safety & Hardware Standards
- Updated Quality Gates for robotics context

Template Consistency Check:
✅ plan-template.md - Constitution Check section aligns with robotics principles
✅ spec-template.md - Safety and performance requirements supported
✅ tasks-template.md - ROS 2 package structure and modularity supported
✅ Existing templates remain compatible with new constitution

Follow-up Actions:
- Validate all existing specs/plans/tasks against new robotics principles
- Update README.md with ROS 2 quickstart and hardware setup
- Establish CI/CD for ROS 2 package builds and simulation tests
-->

# Physical AI & Robotics Engineering Constitution

## Core Principles

### I. Implementation-First Philosophy
Every robotics feature MUST produce:
- **Working implementation**: Runnable code in simulation (Gazebo/Isaac Sim) before physical deployment
- **Spec-compliant artifacts**: Formal specs, plans, tasks, ADRs following Spec-Kit Plus SDD-RI templates
- **Hardware abstraction**: Code that runs identically in simulation and on physical robots (parameter tuning only)

**Rationale**: Robotics projects fail when simulation diverges from reality or documentation doesn't match implementation. Simulation-first development enables rapid iteration while maintaining deployment fidelity.

### II. Accuracy
All technical implementations MUST be correct for current stable versions of:
- Python 3.10+ (primary language)
- ROS 2 Humble/Iron (middleware)
- Gazebo Classic 11 or Gazebo Harmonic (simulation)
- NVIDIA Isaac ROS (perception/navigation)
- Hardware drivers (RealSense SDK, Jetson JetPack 5.1+)

**Rationale**: Robotics systems integrate complex dependencies. Version mismatches cause build failures, runtime crashes, and unpredictable behavior. Pin all versions and test in clean environments.

### III. Safety-First Design
All robot systems MUST implement:
- **Emergency stop**: Physical button halting all motion within 100ms
- **Human detection**: Vision-based person detection with automatic speed limiting (≤0.5 m/s within 1m proximity)
- **Fault handling**: Graceful degradation for sensor failures, API timeouts, network loss
- **Battery management**: Low-battery warnings and safe shutdown procedures
- **Fail-safe defaults**: System defaults to safe state on error (motors disabled, navigation paused)

**Rationale**: Unsafe robots cause injury, property damage, and project termination. Safety must be baked into architecture from day one, not added as afterthought.

### IV. Modularity (ROS 2 Packages)
Each subsystem MUST be:
- **Independent ROS 2 package**: Separately buildable with `colcon build --packages-select <pkg>`
- **Loosely coupled**: Communicates via standard ROS 2 topics/services/actions, not direct function calls
- **Testable in isolation**: Unit tests run without launching entire system
- **Hardware-agnostic**: Simulation and physical hardware use same nodes (only driver swap via launch args)

**Rationale**: Monolithic robotics code is unmaintainable and untestable. ROS 2 package modularity enables parallel development, fault isolation, and incremental deployment.

### V. Sim-to-Real Transferability
Code tested in simulation MUST:
- **Zero logic changes**: Same Python nodes, topic names, message types work on physical hardware
- **Parameter tuning only**: Allow sensor calibration, PID gains, detection thresholds via YAML config files
- **Performance targets**: <20% degradation in task completion time, path accuracy, detection rates
- **Document gaps**: Explicitly note simulation limitations (friction models, sensor noise, network latency)

**Rationale**: Robotics projects die in "sim-to-real hell" when simulation code requires rewrites for hardware. Hardware abstraction layers and disciplined parameter management prevent this.

### VI. Reproducibility
All installations MUST be executable by team members:
- **Dependency lists**: Complete `rosdep` keys, `apt` packages, `pip` requirements with pinned versions
- **Installation scripts**: Tested setup scripts (`quickstart.md` or `install.sh`) verified in clean Ubuntu 22.04
- **Verification commands**: Post-install checks (`ros2 topic list`, `gazebo --version`, camera driver tests)
- **Hardware documentation**: BOM (Bill of Materials), wiring diagrams, calibration procedures for physical robots

**Rationale**: "Works on my machine" is unacceptable when debugging hardware integration. Reproducible environments reduce setup time from days to hours.

### VII. Observable Systems
All robot systems MUST implement:
- **Structured logging**: Python `logging` module at INFO level for key events (commands, state transitions, errors)
- **ROS bag recording**: All topics recorded for replay analysis and debugging
- **Real-time visualization**: RViz dashboards displaying robot state, camera feeds, navigation paths, task status
- **Persistent logs**: Timestamped log files stored for post-mortem analysis and performance metrics

**Rationale**: Robotics bugs are Heisenbugs—they disappear when you try to debug them. Comprehensive logging and recording enable after-the-fact root cause analysis.

### VIII. Performance Constraints
Systems MUST meet specified targets:
- **Inference latency**: <100ms for LLM parsing, <200ms for vision processing
- **Control frequency**: ≥10Hz for navigation commands, ≥30Hz for joint control
- **Safety response**: <100ms emergency stop latency from button press to motor halt
- **Resource budgets**: Memory limits (Jetson 8GB), compute (GPU/CPU utilization <80%), power (battery runtime)

**Rationale**: Robotics is real-time systems engineering. Missed deadlines cause motion jitter, unsafe behavior, and task failures. Performance budgets must be measurable and enforced.

### IX. SDD-RI Alignment
All development workflows (planning, spec'ing, tasking) MUST:
- Follow Spec-Kit Plus command patterns (`/sp.*` commands)
- Generate compliant specs, plans, and task files
- Create PHRs (Prompt History Records) for all significant AI interactions
- Suggest ADRs when architecturally significant decisions are made

**Rationale**: Disciplined specification-driven development prevents scope creep and ensures traceability from requirements to implementation. PHRs and ADRs create audit trail for technical decisions.

## Robotics Project Standards

### ROS 2 Package Structure (MANDATORY)
Each package MUST contain:
1. **package.xml**: Dependencies, maintainers, licenses (ament_python or ament_cmake)
2. **CMakeLists.txt** (for ament_cmake) or **setup.py** (for ament_python)
3. **Launch files**: `launch/` directory with `.launch.py` files for node orchestration
4. **Config files**: `config/` directory with YAML parameter files
5. **Source code**: `src/` (C++) or `<package_name>/` (Python) with documented modules
6. **README.md**: Package purpose, dependencies, launch instructions, parameter documentation

**Length Constraints**:
- Maximum: 2000 lines per node (beyond this, split into multiple nodes)
- Minimum: 100 lines (below this, merge into parent package)

### Code Standards
All ROS 2 code MUST:
- Include file paths as comments at top (e.g., `# perception/perception/object_detector_node.py`)
- Use modern Python 3.10+ syntax (type hints, dataclasses, async where applicable)
- Pass linting (ruff or pylint for Python, clang-format for C++)
- Include error handling for all ROS 2 API calls (QoS failures, service timeouts)
- Avoid deprecated ROS 2 APIs (use rclpy.node.Node, not rclpy.create_node)

### Launch File Standards
- **Composable nodes**: Use component containers for low-latency communication
- **Parameterization**: All hardware-specific values in YAML files, not hardcoded
- **Conditional logic**: Separate launch files for simulation vs. physical hardware (`use_sim_time` parameter)
- **Namespace isolation**: Use ROS 2 namespaces (`/robot1/`) to support multi-robot systems

### Documentation Standards
- **User Stories**: Given-When-Then acceptance scenarios for each feature (spec.md)
- **Architecture diagrams**: ROS 2 node graphs exported from `rqt_graph`, URDF visualizations from RViz
- **API documentation**: Service/action interfaces documented in `contracts/` with request/response examples
- **Deployment guides**: Step-by-step instructions in `quickstart.md` or `README.md`

## Safety & Hardware Standards

### Emergency Stop Requirements (MANDATORY)
- Physical button wired to GPIO or ROS 2 topic `/emergency_stop`
- Halts all actuators within 100ms (measured, not estimated)
- Publishes `/estop_status` (std_msgs/Bool) at ≥5Hz
- Triggers graceful shutdown of task execution (pause, not abort mid-motion)

### Human Detection Requirements (MANDATORY)
- Vision-based person detection (YOLO, MoveNet, or equivalent) at ≥10Hz
- Speed limiting: ≤0.5 m/s when human within 1m radius
- Path re-planning: Avoid planned paths intersecting human bounding boxes
- Audible warnings: Announce "Human detected" via text-to-speech

### Sensor Calibration Requirements
- Camera intrinsics: Store calibration YAML from `camera_calibration` package
- IMU bias: Stationary robot calibration for gyro/accelerometer bias
- Force sensors: Weight calibration with known masses (50g, 100g, 250g, 500g test objects)
- Odometry: Measure and document wheel slip, encoder noise, drift rates

### Hardware Documentation
- **Bill of Materials (BOM)**: Part numbers, suppliers, costs for all components
- **Wiring diagrams**: Annotated photos or Fritzing diagrams for power, CAN bus, USB connections
- **Assembly instructions**: Step-by-step with photos for robot construction
- **Troubleshooting guides**: Common hardware failures and diagnostic procedures

## Quality Gates

### Gate 1: Constitution Compliance (BLOCKING)
**When**: Before any implementation begins
**Check**: Spec/plan aligns with all nine Core Principles
**Owner**: Claude Code / Human Reviewer
**Failure Action**: Revise spec; do not proceed to tasking/implementation

### Gate 2: Build Success (BLOCKING)
**When**: After any package changes
**Check**: `colcon build` exits 0, no warnings, all tests pass
**Owner**: CI/CD pipeline (GitHub Actions or local pre-commit hook)
**Failure Action**: Fix build errors before commit; broken builds block all merges

### Gate 3: Simulation Tests (BLOCKING)
**When**: After implementing user story
**Check**: Task succeeds ≥80% in Gazebo over 10 trials
**Owner**: Manual testing or automated simulation CI
**Failure Action**: Debug failures, tune parameters, document known issues before hardware deployment

### Gate 4: Safety Validation (BLOCKING)
**When**: Before first physical robot test
**Check**: E-stop halts motion <100ms, human detection triggers speed limit, battery management prevents deep discharge
**Owner**: Hardware team manual testing
**Failure Action**: Do not power robot until all safety systems verified

### Gate 5: Sim-to-Real Gap (ADVISORY)
**When**: After physical robot testing
**Check**: Performance degradation <20% vs. simulation (completion time, path accuracy, detection rates)
**Owner**: Hardware team measurement
**Failure Action**: Document gap; acceptable if >80% success rate maintained; blocking if safety compromised

### Gate 6: Performance Profiling (ADVISORY)
**When**: After integration testing
**Check**: Latencies meet targets (<100ms LLM, <200ms vision), CPU/GPU <80%, memory <90%
**Owner**: Profiling tools (ros2 topic hz, top, nvidia-smi)
**Failure Action**: Optimize critical path if targets missed; advisory unless safety affected

## Governance

### Amendment Process
1. **Propose**: Submit amendment with rationale and impact analysis
2. **Review**: Assess affected packages, launch files, and hardware dependencies
3. **Version Bump**: Apply semantic versioning (see below)
4. **Synchronize**: Update all dependent files (templates, existing packages, documentation)
5. **Announce**: Document in CHANGELOG.md; notify all team members

### Versioning Policy
- **MAJOR (X.0.0)**: Principle removal, redefinition, or incompatible safety/architecture change
- **MINOR (0.X.0)**: New principle added, new quality gate, expanded requirements
- **PATCH (0.0.X)**: Clarifications, typo fixes, wording improvements

### Compliance Review
- **Frequency**: After each user story implementation or bi-weekly (whichever is sooner)
- **Scope**: Random sample 20% of packages + all recently changed nodes
- **Action**: Log violations in GitHub issues; require fix before next sprint
- **Escalation**: 3+ violations in same package = full package refactor required

### Living Document Status
This constitution supersedes all prior practices and informal agreements. When constitution conflicts with specs, plans, or implementation, **constitution wins**—update the artifacts to comply.

Complexity (new dependencies, custom hardware, additional sensors) requires explicit justification against this constitution. Default answer to "Should we add [X]?" is **no** unless it directly supports a Core Principle or safety requirement.

**Version**: 2.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05 | **Domain**: Physical AI & Robotics Engineering
