
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare the LiDAR topic for both sensors
        DeclareLaunchArgument('lidar1_topic', default_value='/lidar1/scan', description='Topic for LiDAR 1 scan'),
        DeclareLaunchArgument('lidar2_topic', default_value='/lidar2/scan', description='Topic for LiDAR 2 scan'),
        
        # Node for LDLiDAR 1 driver
        Node(
            package='ldlidar_ros2',
            executable='ldlidar_node',
            name='ldlidar_node1',
            output='screen',
            parameters=[{'lidar_topic': '/lidar1/scan'}],
            remappings=[('/scan', '/lidar1/scan')]
        ),

        # Node for LDLiDAR 2 driver
        Node(
            package='ldlidar_ros2',
            executable='ldlidar_node',
            name='ldlidar_node2',
            output='screen',
            parameters=[{'lidar_topic': '/lidar2/scan'}],
            remappings=[('/scan', '/lidar2/scan')]
        ),
        
        # Node for LiDAR Clustering
        Node(
            package='lidar_clustering',
            executable='lidar_clustering_node',
            name='lidar_clustering_node',
            output='screen',
            parameters=[],
            remappings=[('/scan1', '/lidar1/scan'),
                        ('/scan2', '/lidar2/scan'),
                        ('/cluster_output', '/lidar_clusters')]
        ),
    ])
