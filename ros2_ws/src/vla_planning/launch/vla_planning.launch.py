"""
VLA Planning System Launch File

Launches all VLA planning nodes for voice-commanded task execution.

Nodes launched:
- audio_capture_node: Captures voice input with VAD
- whisper_asr_node: Transcribes audio to text
- tts_node: Provides voice feedback
- vla_planner_node: Parses commands with LLM
- task_executor_node: Coordinates multi-step execution

Usage:
  ros2 launch vla_planning vla_planning.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='vla_params.yaml',
        description='VLA configuration file'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    # Audio Capture Node
    audio_capture_node = Node(
        package='vla_planning',
        executable='audio_capture_node',
        name='audio_capture',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Whisper ASR Node
    whisper_asr_node = Node(
        package='vla_planning',
        executable='whisper_asr_node',
        name='whisper_asr',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # TTS Node
    tts_node = Node(
        package='vla_planning',
        executable='tts_node',
        name='tts',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # VLA Planner Node
    vla_planner_node = Node(
        package='vla_planning',
        executable='vla_planner_node',
        name='vla_planner',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Task Executor Node
    task_executor_node = Node(
        package='vla_planning',
        executable='task_executor_node',
        name='task_executor',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        audio_capture_node,
        whisper_asr_node,
        tts_node,
        vla_planner_node,
        task_executor_node,
    ])
