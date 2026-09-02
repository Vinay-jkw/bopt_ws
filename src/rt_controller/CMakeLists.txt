cmake_minimum_required(VERSION 3.8)
project(rt_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rt_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED) 
find_package(trajectory_msgs REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(twist_relay src/twist_relay.cpp)
ament_target_dependencies(twist_relay rclcpp geometry_msgs rt_interfaces)

add_executable(rt_key src/rt_key.cpp)
ament_target_dependencies(rt_key rclcpp geometry_msgs std_msgs rt_interfaces sensor_msgs nav_msgs tf2_ros tf2 ${Eigen_LIBRARIES})

add_executable(rt_nmpc_controller src/rt_nmpc_controller.cpp)
ament_target_dependencies(rt_nmpc_controller rclcpp geometry_msgs std_msgs rt_interfaces sensor_msgs nav_msgs tf2_ros tf2 ${Eigen_LIBRARIES})

add_executable(rt_simple src/rt_simple.cpp)
ament_target_dependencies(rt_simple rclcpp geometry_msgs trajectory_msgs std_msgs rt_interfaces sensor_msgs nav_msgs tf2_ros tf2 ${Eigen_LIBRARIES})

add_executable(manipulator_service_node src/rt_hydraulics_service.cpp)
ament_target_dependencies(manipulator_service_node rclcpp geometry_msgs std_msgs rt_interfaces sensor_msgs nav_msgs tf2_ros tf2 ${Eigen_LIBRARIES})

add_executable(rt_odom src/rt_odom.cpp)
ament_target_dependencies(rt_odom
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(rt_current_pose src/rt_current_pose.cpp)
ament_target_dependencies(rt_current_pose
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  rt_key
  twist_relay
  rt_simple
  rt_odom
  rt_current_pose
  rt_nmpc_controller
  manipulator_service_node    
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY src launch config
  DESTINATION share/${PROJECT_NAME}
)

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
