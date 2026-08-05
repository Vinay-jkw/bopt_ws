import os 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run Gazebo headless'
    )

    package_path = get_package_share_directory('bopt_description')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_path, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items()
    )

    rviz_config_file = os.path.join(
        package_path,
        "rviz",
        "display.rviz"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    return LaunchDescription([
        gui_arg,
        gazebo,
        rviz
    ])
