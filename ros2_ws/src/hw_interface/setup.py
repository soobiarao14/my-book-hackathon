from setuptools import setup, find_packages

package_name = 'hw_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hw_interface.launch.py']),
        ('share/' + package_name + '/config', ['config/hardware_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Capstone Team',
    maintainer_email='capstone@example.com',
    description='Hardware interface for robot actuators and sensors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver_node = hw_interface.motor_driver_node:main',
            'sensor_fusion_node = hw_interface.sensor_fusion_node:main',
            'battery_monitor_node = hw_interface.battery_monitor_node:main',
            'estop_node = hw_interface.estop_node:main',
        ],
    },
)
