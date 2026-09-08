import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'workflow_node'

setup(
    name=package_name,
    version='0.0.0',
    # Automatically discovers workflow_node and all sub-packages
    # (config, states, handlers, planners, communication, utils)
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install YAML configs to share so ament_index_python can locate them
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        # data/ is a runtime-writable directory; nothing to install from it
        # (constructed_rs_path.pkl is written here at runtime, not shipped)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='AGV workflow orchestration node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'workflow_node = workflow_node.workflow_node:main',
        ],
    },
)
