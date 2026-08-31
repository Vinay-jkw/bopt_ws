from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config_file  = LaunchConfiguration("config_file")
    declare_config_file = DeclareLaunchArgument(
        "config_file",
        default_value='',
        description=''
    )
    return LaunchDescription([
        declare_config_file,
        Node(
            package='ros2_mqtt_bridge',
            executable='mqtt_bridge',
            name='mqtt_bridge',
            output='screen',  
            emulate_tty=True,  
            arguments=['--config-file', config_file] 
            # arguments=['--ros-args', '--log-level', 'INFO'] 
        )
    ])