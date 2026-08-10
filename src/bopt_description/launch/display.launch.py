import os
import xacro
from launch import LaunchDescription #it tell ros2 how to launch the file
from launch_ros.actions import Node # start the executable node in the packag

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_path = get_package_share_directory('bopt_description') #ye path dega absoulute path package ka

    #ab urdf ko find karna hai 
    urdf_file = os.path.join(
        package_path, 
        "urdf",
        "robot.urdf.xacro"
    )
    robot_description_config = xacro.process_file(urdf_file)

    robot_description = {
        "robot_description": robot_description_config.toxml()
    }
    #ab robot state publisher ko launch karna hai
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher', 
        output='screen',
        parameters=[robot_description]
    )

    #ab rviz ko launch karna hai
    rviz_config_file = os.path.join(
        package_path,
        "rviz",
        "display.rviz"
    )
    
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz
    ])


    
