# HardwareState.msg
# Hardware status monitoring message
# Reference: specs/001-humanoid-robotics-capstone/data-model.md

std_msgs/Header header
float64[] joint_positions       # Joint positions (radians/meters)
float64[] joint_velocities      # Joint velocities
float32[] motor_temperatures    # Temperature per motor (Celsius)
float32 battery_voltage         # Current battery voltage
bool estop_status               # Emergency stop status (true = pressed)
bool camera_health              # Camera operational status
bool imu_health                 # IMU sensor health
