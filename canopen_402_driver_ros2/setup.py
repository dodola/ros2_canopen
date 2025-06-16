from setuptools import setup
import os
from glob import glob

package_name = 'canopen_402_driver_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'resource'), glob(os.path.join('resource', '*.eds'))), # Install EDS files
    ],
    install_requires=['setuptools', 'rclpy', 'canopen', 'sensor_msgs', 'std_srvs', 'diagnostic_msgs', 'canopen_interfaces'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='dev@example.com',
    description='ROS 2 node for CiA 402 CANopen driver',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Corrected entry point to match the new lifecycle node class name if it changes
            # For now, assuming the executable name remains cia402_ros_node which points to a main in that file
            'cia402_ros_node = canopen_402_driver_ros2.cia402_ros_node:main',
        ],
    },
)
