# TaskPlan.msg
# Represents parsed multi-step task with execution state
# Reference: specs/001-humanoid-robotics-capstone/data-model.md

std_msgs/Header header
string plan_id                  # UUID for tracking
string original_command         # Raw voice command
int32 total_subtasks            # Number of steps
string[] subtask_actions        # Array of action types
string[] subtask_params         # JSON serialized parameters per subtask
int32 current_subtask_index     # Currently executing step (0-indexed)
bool[] subtask_completion       # True if subtask succeeded
string[] subtask_errors         # Error messages per subtask (empty if success)
uint8 status                    # IDLE=0, EXECUTING=1, SUCCESS=2, FAILED=3, PAUSED=4
float32 estimated_time_remaining  # Seconds (for battery check FR-013)
