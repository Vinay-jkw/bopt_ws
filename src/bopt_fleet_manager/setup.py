from setuptools import setup
from glob import glob
import os

package_name = 'bopt_fleet_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BOPT Fleet Team',
    maintainer_email='maintainer@example.com',
    description='Read-only fleet monitor for BOPT Gazebo robots.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fleet_monitor = bopt_fleet_manager.fleet_monitor:main',
        ],
    },
)
