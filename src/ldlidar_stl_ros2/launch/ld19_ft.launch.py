#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

'''
Parameter Description:
---
- Set laser scan directon: 
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
'''

def generate_launch_description():
  # LDROBOT LiDAR publisher node
  ldlidar_Lidar_LFT = Node(
      package='ldlidar_stl_ros2',
      executable='ldlidar_stl_ros2_node',
      name='LD19',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD19'},
        {'topic_name': 'Lidar_LFT'},     
        {'frame_id': 'Lidar_LFT'},
        {'port_name': '/dev/Lidar_LFT'},
        {'port_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},
        {'angle_crop_max': 225.0}
      ]
  )

    # base_link to rear_laser tf node
  base_link_to_Lidar_LFT_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_Lidar_LFT',
    arguments=['0.43','0.19','0.0','-1.5707963267','3.1415926535','0','load_wheel_base_link','Lidar_LFT']
    # arguments=['0.135','0.19','0.0','-1.5707963267','3.1415926535','0','load_wheel_base_link','Lidar_LFT']
  )

  ldlidar_Lidar_RFT = Node(
      package='ldlidar_stl_ros2',
      executable='ldlidar_stl_ros2_node',
      name='LD19',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD19'},
        {'topic_name': 'Lidar_RFT'},
        {'frame_id': 'Lidar_RFT'},
        {'port_name': '/dev/Lidar_RFT'},
        {'port_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 35.0},
        {'angle_crop_max': 130.0}
      ]
  )

  # base_link to right_laser tf node
  base_link_to_Lidar_RFT_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_Lidar_RFT',
    arguments=['0.43','-0.19','0.0','-1.5707963267','3.1415926535','0','load_wheel_base_link','Lidar_RFT']
    # arguments=['0.135','-0.19','0.0','-1.5707963267','3.1415926535','0','load_wheel_base_link','Lidar_RFT']
  )

  # # base_link to right_laser tf node
  # base_link_to_Lidar_RFT2_tf_node = Node(
  #   package='tf2_ros',
  #   executable='static_transform_publisher',
  #   name='base_link_to_Lidar_RFT2',
  #   arguments=['0.135','-0.19','0.0','-1.5707963267','3.1415926535','0','load_wheel_base_link','Lidar_RFT2']
  # )

  forktip_node_ds = Node(
    package='forktip_lidar_detection',
    namespace='',
    executable='forktip_lidar_node_ds',
    name='forktip_lidar_node_ds'
  )

  forktip_node_pap = Node(
    package='forktip_lidar_detection',
    namespace='',
    executable='forktip_lidar_node_pap',
    name='forktip_lidar_node_pap'
  )
  
  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(ldlidar_Lidar_LFT)
  ld.add_action(ldlidar_Lidar_RFT)

  ld.add_action(base_link_to_Lidar_LFT_tf_node)
  ld.add_action(base_link_to_Lidar_RFT_tf_node)
  ld.add_action(forktip_node_ds)
  ld.add_action(forktip_node_pap)


  return ld

