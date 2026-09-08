from setuptools import setup, find_packages
from glob import glob

package_name = 'nmpc_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['nmpc_controller/config/nmpc_config.yaml',
             'nmpc_controller/config/nmpc_config_long_fork.yaml']),
        ('share/' + package_name + '/data',
            glob('nmpc_controller/data/*.pkl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jkw',
    maintainer_email='jkw@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = nmpc_controller.main:main',
        ],
    },
)
