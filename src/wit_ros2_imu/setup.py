from setuptools import setup
import os
from glob import glob

package_name = 'wit_ros2_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ament index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # launch files (ONLY HERE)
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # config files (EKF, AMCL, etc.)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='IMU + EKF bringup',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wit_ros2_imu = wit_ros2_imu.wit_ros2_imu:main',
            'imu_transformer = wit_ros2_imu.imu_transformer:main',
            'yaw_comparator = wit_ros2_imu.yaw_comparator:main',
        ],
    },
)
