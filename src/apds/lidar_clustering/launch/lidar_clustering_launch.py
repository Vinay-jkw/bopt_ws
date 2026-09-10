
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare the LiDAR topic for both sensors
        DeclareLaunchArgument('lidar1_topic', default_value='/Lidar_LFT', description='Topic for Left LiDAR scan'),
        DeclareLaunchArgument('lidar2_topic', default_value='/Lidar_RFT', description='Topic for Right LiDAR scan'),
        
        # Node for LiDAR Clustering
        Node(
            package='lidar_clustering',
            executable='lidar_clustering_node',
            name='lidar_clustering_node',
            output='screen',
            parameters=[],
            remappings=[('/scan1', '/Lidar_LFT'),
                        ('/scan2', '/Lidar_RFT'),
                        ('/cluster_output', '/lidar_clusters')]
        ),
    ])
