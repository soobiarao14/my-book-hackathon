from setuptools import setup, find_packages

package_name = 'navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigation.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Capstone Team',
    maintainer_email='capstone@example.com',
    description='Navigation system using Visual SLAM and Nav2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vslam_node = navigation.vslam_node:main',
            'path_planner_node = navigation.path_planner_node:main',
            'obstacle_monitor_node = navigation.obstacle_monitor_node:main',
        ],
    },
)
