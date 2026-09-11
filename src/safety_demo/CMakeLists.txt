cmake_minimum_required(VERSION 3.8)
project(safety_demo)

# Add compiler options for warnings and C++17 standard
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sqlite3_vendor REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(bopt_interfaces REQUIRED)

# Include directories
include_directories(include)

# Create library with all safety components
add_library(safety_demo_lib
  src/utils/database_manager.cpp
  src/detectors/safety_zone_detector.cpp
  src/interfaces/velocity_controller.cpp
  src/interfaces/visualization_publisher.cpp
  src/utils/error_handler.cpp
  src/utils/policy_selector.cpp
  src/core/safety_node.cpp
)

# Link dependencies to library
ament_target_dependencies(safety_demo_lib
  rclcpp
  visualization_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
  bopt_interfaces
)

# Link SQLiteCpp and sqlite3 libraries
target_link_libraries(safety_demo_lib
  sqlite3
)

# Add executable
add_executable(safety_node_viz src/nodes/main.cpp)

# Link executable to library
target_link_libraries(safety_node_viz safety_demo_lib)

# Install the executable
install(TARGETS
    safety_node_viz
    DESTINATION lib/${PROJECT_NAME})

# Install headers
install(DIRECTORY include/
    DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch)

# Install config files
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config)

# Add optional testing
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

# Finalize the ament package
ament_package()
