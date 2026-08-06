from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bopt_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vinay',
    maintainer_email='vinay@jkwinnovatics.com',
    description='High-level controller package for the BOPT robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = bopt_controller.robot_controller_node:main',
            'teleop_keyboard   = bopt_controller.teleop_keyboard:main',
        ],
    },
)
