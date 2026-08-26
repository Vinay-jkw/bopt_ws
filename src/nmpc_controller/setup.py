from setuptools import setup

package_name = 'nmpc_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fbots',
    maintainer_email='fbots@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nmpc_controller = nmpc_controller.nmpc_controller:main',
            'nmpc_controller_v2 = nmpc_controller.nmpc_controller_v2:main',
            'nmpc_controller_v2_fast = nmpc_controller.nmpc_controller_v2_fast:main',
            'nmpc_controller_v2_slow = nmpc_controller.nmpc_controller_v2_slow:main',
            'nmpc_controller_v2_dp = nmpc_controller.nmpc_controller_v2_dp:main',#deep pickup
            'nmpc_controller_v2_dd = nmpc_controller.nmpc_controller_v2_dd:main',#deep drop
            'nmpc_controller_v2_pp = nmpc_controller.nmpc_controller_v2_pp:main',#pallet pickup
            'nmpc_controller_v2_park = nmpc_controller.nmpc_controller_v2_park:main',#park

        ],
    },
)
