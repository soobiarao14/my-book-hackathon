from setuptools import setup, find_packages

package_name = 'vla_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vla_planning.launch.py']),
        ('share/' + package_name + '/config', ['config/vla_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Capstone Team',
    maintainer_email='capstone@example.com',
    description='Vision-Language-Action planning system for voice-commanded robotics',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_capture_node = vla_planning.audio_capture_node:main',
            'whisper_asr_node = vla_planning.whisper_asr_node:main',
            'tts_node = vla_planning.tts_node:main',
            'vla_planner_node = vla_planning.vla_planner_node:main',
            'task_executor_node = vla_planning.task_executor_node:main',
        ],
    },
)
