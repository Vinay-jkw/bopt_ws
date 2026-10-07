import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg = get_package_share_directory('costmap_to_laserscan')
    default_params = os.path.join(pkg, 'config', 'costmap_to_laserscan.yaml')

    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    use_composition = LaunchConfiguration('use_composition')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'use_composition', default_value='false',
            description='Run inside a container so the costmap never crosses a process '
                        'boundary (needs costmap_transient_local_qos:=false to actually '
                        'take the intra-process path).'),

        Node(
            package='costmap_to_laserscan',
            executable='costmap_to_laserscan',
            name='costmap_to_laserscan',
            namespace=namespace,
            output='screen',
            parameters=[params_file],
            condition=UnlessCondition(use_composition),
        ),

        ComposableNodeContainer(
            name='costmap_to_laserscan_container',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container_isolated',
            output='screen',
            arguments=['--use_intra_process_comms'],
            composable_node_descriptions=[
                ComposableNode(
                    package='costmap_to_laserscan',
                    plugin='costmap_to_laserscan::CostmapToLaserScanNode',
                    name='costmap_to_laserscan',
                    parameters=[params_file],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            condition=IfCondition(use_composition),
        ),
    ])