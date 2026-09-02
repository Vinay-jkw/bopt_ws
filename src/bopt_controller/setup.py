from setuptools import find_packages, setup

package_name = 'bopt_controller'

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
            'share/' + package_name + '/launch',
            ['launch/controller.launch.py']
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
           'bopt_controller = bopt_controller.controller_node:main',
           'bopt_keyboard = bopt_controller.teleop_keyboard:main',
           'odometry_node = bopt_controller.odometry_node:main',
           'bopt_key = bopt_controller.bopt_key_node:main',
           'bopt_twist_relay = bopt_controller.bopt_twist_relay:main',
           'bopt_main_controller = bopt_controller.bopt_main_controller:main',
           'bopt_hydraulic_controller = bopt_controller.bopt_hydraulic_controller:main',
        ],
    },
)
