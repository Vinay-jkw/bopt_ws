from setuptools import find_packages, setup

package_name = 'forktip_lidar_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yugvir',
    maintainer_email='yugvir@futuristicbots.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forktip_lidar_node_ds = forktip_lidar_detection.forktip_lidar_node_ds:main',
            'forktip_lidar_node_pap = forktip_lidar_detection.forktip_lidar_node_pap:main',
            'forktip_lidar_node = forktip_lidar_detection.forktip_lidar_node:main',

        ],
    },
)
