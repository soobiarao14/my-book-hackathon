"""
Full System Simulation Launch File

Launches complete robotics stack in Gazebo simulation.

System components:
- Gazebo Classic 11 simulation environment
- Custom interfaces
- VLA planning (voice, ASR, LLM, task execution)
- Perception (camera, object/human detection)
- Navigation (VSLAM, path planning, obstacle avoidance)
- Manipulation (grasp planning, arm control)
- Hardware interface (simulated motors, sensors, battery)

Usage:
  ros2 launch simulation.launch.py

Prerequisites:
  - Gazebo models in gazebo_ws/models/
  - World file in gazebo_ws/worlds/office_environment.world
  - URDF robot model configured
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Get package paths
    vla_planning_pkg = FindPackageShare('vla_planning')
    perception_pkg = FindPackageShare('perception')
    navigation_pkg = FindPackageShare('navigation')
    manipulation_pkg = FindPackageShare('manipulation')
    hw_interface_pkg = FindPackageShare('hw_interface')

    # Launch Gazebo with world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose',
             'C:/Users/Since Tech/my-book/gazebo_ws/worlds/office_environment.world',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Spawn robot URDF in Gazebo
    # TODO: Add spawn_entity node for robot URDF

    # VLA Planning System
    vla_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([vla_planning_pkg, 'launch', 'vla_planning.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Perception System
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([perception_pkg, 'launch', 'perception.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Navigation System
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([navigation_pkg, 'launch', 'navigation.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Manipulation System
    manipulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([manipulation_pkg, 'launch', 'manipulation.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Hardware Interface (Simulated)
    hw_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([hw_interface_pkg, 'launch', 'hw_interface.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        gazebo,
        vla_planning_launch,
        perception_launch,
        navigation_launch,
        manipulation_launch,
        hw_interface_launch,
    ])
