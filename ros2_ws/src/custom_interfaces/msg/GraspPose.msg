# GraspPose.msg
# Grasp pose for manipulation
# Reference: specs/001-humanoid-robotics-capstone/data-model.md

std_msgs/Header header
geometry_msgs/Pose pose         # 3D position and orientation of end-effector
float32 gripper_width           # Gripper opening width (meters)
float32 grasp_force             # Required grasp force (Newtons)
geometry_msgs/Vector3 approach_vector  # Approach direction
