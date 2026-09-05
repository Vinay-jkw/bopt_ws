import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bopt_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jkw',
    maintainer_email='vinaysharma17005@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'current_pose_publisher = bopt_localization.current_pose_publisher:main',
            'imu_covariance_relay = bopt_localization.imu_covariance_relay:main',
        ],
    },
)
