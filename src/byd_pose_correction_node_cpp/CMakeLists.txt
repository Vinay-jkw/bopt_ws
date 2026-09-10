cmake_minimum_required(VERSION 3.8)
project(byd_pose_correction_node_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(last_amcl_pose_publisher src/last_amcl_pose_publisher.cpp)
ament_target_dependencies(last_amcl_pose_publisher rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(pose_correction_node src/pose_correction_node.cpp)
ament_target_dependencies(pose_correction_node rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(precise_control_node src/precise_control_node.cpp)
ament_target_dependencies(precise_control_node rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(precise_control_node_with_pallet_check src/precise_control_node_with_pallet_check.cpp)
ament_target_dependencies(precise_control_node_with_pallet_check rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(precise_control_node_with_forktip_check src/precise_control_node_with_forktip_check.cpp)
ament_target_dependencies(precise_control_node_with_forktip_check rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(precise_control_node_with_forktip_check_for_drop src/precise_control_node_with_forktip_check_for_drop.cpp)
ament_target_dependencies(precise_control_node_with_forktip_check_for_drop rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(precise_control_node_v2 src/precise_control_node_v2.cpp)
ament_target_dependencies(precise_control_node_v2 rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(precise_control_node_ft src/precise_control_node_ft.cpp)
ament_target_dependencies(precise_control_node_ft rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(pose_correction_node_rad src/pose_correction_node_rad.cpp)
ament_target_dependencies(pose_correction_node_rad rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs tf2_ros)





install(TARGETS
  last_amcl_pose_publisher
  precise_control_node
  precise_control_node_ft
  precise_control_node_v2
  pose_correction_node
  pose_correction_node_rad
  precise_control_node_with_pallet_check
  precise_control_node_with_forktip_check
  precise_control_node_with_forktip_check_for_drop
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
