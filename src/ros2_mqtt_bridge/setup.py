from setuptools import setup
import os
from glob import glob

package_name = 'ros2_mqtt_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line installs all files from the launch directory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shubhisingh',
    maintainer_email='shubhi@futuristicsbots.com',
    description='ROS 2 to MQTT bridge for robot state communication',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_bridge = ros2_mqtt_bridge.mqtt_bridge:main',
        ],
    },
)
