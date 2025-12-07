from setuptools import setup, find_packages

package_name = 'manipulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/manipulation.launch.py']),
        ('share/' + package_name + '/config', ['config/moveit_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Capstone Team',
    maintainer_email='capstone@example.com',
    description='Manipulation system using MoveIt 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_planner_node = manipulation.grasp_planner_node:main',
            'arm_controller_node = manipulation.arm_controller_node:main',
        ],
    },
)
