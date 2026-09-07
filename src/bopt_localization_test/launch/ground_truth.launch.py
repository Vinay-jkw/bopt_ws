from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ground_truth_node = Node(
        package='bopt_localization_test',
        executable='ground_truth_node',
        name='ground_truth_node',
        output='screen'
    )

    return LaunchDescription([
        ground_truth_node
    ])

