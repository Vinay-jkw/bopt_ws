from setuptools import setup
from glob import glob
import os

package_name = 'workflow_node'

def existing_file(path):
    """Return [path] if file exists, else an empty list. No prints allowed."""
    return [path] if os.path.isfile(path) else []

def existing_glob(pattern):
    """Return matched files if any, else empty list. No prints."""
    files = glob(pattern)
    return files if files else []


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    data_files=[
        # Required ROS2 files
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),

        # Optional YAML config
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),

        # Optional map_details files
        (
            os.path.join('share', package_name, 'map_details'),
            glob('map_details/*')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'workflow_node = workflow_node.workflow_node:main',
            'workflow_node_v2 = workflow_node.workflow_node_v2:main'
        ],
    },
)
