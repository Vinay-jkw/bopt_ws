from setuptools import find_packages, setup

package_name = 'lidar_clustering'

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
    maintainer='ashu',
    maintainer_email='siddhi@futuristicbots.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_clustering_node = lidar_clustering.lidar_clustering_combined:main',
            'lidar_clustering_node_once = lidar_clustering.lidar_clustering_combined_once:main'

        ],
    },
)
