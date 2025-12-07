from setuptools import setup, find_packages

package_name = 'perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py']),
        ('share/' + package_name + '/config', ['config/perception_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Capstone Team',
    maintainer_email='capstone@example.com',
    description='Perception system for object and human detection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realsense_node = perception.realsense_node:main',
            'object_detector_node = perception.object_detector_node:main',
            'human_detector_node = perception.human_detector_node:main',
        ],
    },
)
