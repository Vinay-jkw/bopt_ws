import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run Gazebo headless'
    )

    package_path = get_package_share_directory('bopt_description')
    
    world_path=os.path.join(
        package_path,
        "worlds",
        "empty.world"
    )

    urdf_file = os.path.join(
        package_path,
        "urdf",
        "robot.urdf.xacro"
    )

    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {
        "robot_description": robot_description_config.toxml()
    }
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        name='joint_state_publisher_gui',
        parameters=[robot_description]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    gazebo_launch=IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),

        launch_arguments={'world': world_path, 'gui': LaunchConfiguration('gui')}.items()
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            "-entity", "bopt_robot",
            "-topic", "robot_description",
            "-x", "0",
            "-y", "0",
            "-z", "0.5"
        ],
        output='screen'
    )

    return LaunchDescription(
        [
            gui_arg,
            robot_state_publisher_node,
            gazebo_launch,
            joint_state_publisher_node,
            spawn_entity
        ]
    )
