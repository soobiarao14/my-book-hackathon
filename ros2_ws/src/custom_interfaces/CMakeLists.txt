cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VoiceCommand.msg"
  "msg/TaskPlan.msg"
  "msg/HardwareState.msg"
  "msg/GraspPose.msg"
  "srv/DetectObjects.srv"
  "srv/ComputeGrasp.srv"
  "action/PickObject.action"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    vision_msgs
)

ament_package()
