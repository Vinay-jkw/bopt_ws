cmake_minimum_required(VERSION 3.8)
project(costmap_to_laserscan)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # The ray-cast kernel benefits from full optimisation, but NEVER enable -ffast-math:
  # the traversal relies on well-behaved infinities for axis-aligned beams and the scan
  # contract relies on inf meaning "no return".
  add_compile_options($<$<CONFIG:Release>:-O3>)
endif()

# Off by default: -march=native breaks binary portability across a heterogeneous fleet.
option(COSTMAP_TO_LASERSCAN_NATIVE "Build for the local micro-architecture" OFF)
option(COSTMAP_TO_LASERSCAN_OPENMP "Parallelise the ray-cast across beams" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  map_msgs
  nav2_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_library(costmap_to_laserscan_component SHARED
  src/costmap_to_laserscan_node.cpp
)
target_include_directories(costmap_to_laserscan_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(costmap_to_laserscan_component ${dependencies})

if(COSTMAP_TO_LASERSCAN_NATIVE)
  target_compile_options(costmap_to_laserscan_component PRIVATE -march=native -mtune=native)
endif()

if(COSTMAP_TO_LASERSCAN_OPENMP)
  find_package(OpenMP REQUIRED)
  target_link_libraries(costmap_to_laserscan_component OpenMP::OpenMP_CXX)
  target_compile_definitions(costmap_to_laserscan_component PRIVATE COSTMAP_TO_LASERSCAN_OPENMP)
endif()

rclcpp_components_register_node(costmap_to_laserscan_component
  PLUGIN "costmap_to_laserscan::CostmapToLaserScanNode"
  EXECUTABLE costmap_to_laserscan
)

install(TARGETS costmap_to_laserscan_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_raycast test/test_raycast.cpp)
  target_link_libraries(test_raycast costmap_to_laserscan_component)
endif()

ament_export_include_directories(include)
ament_export_libraries(costmap_to_laserscan_component)
ament_export_dependencies(${dependencies})
ament_package()