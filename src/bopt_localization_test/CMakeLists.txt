cmake_minimum_required(VERSION 3.8)

project(bopt_localization_test)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(PkgConfig REQUIRED)

pkg_check_modules(IGNITION_TRANSPORT REQUIRED ignition-transport11)
pkg_check_modules(IGNITION_MSGS REQUIRED ignition-msgs8)

add_executable(ground_truth_node
  src/ground_truth_node.cpp
)

target_include_directories(
  ground_truth_node
  PRIVATE
  ${IGNITION_TRANSPORT_INCLUDE_DIRS}
  ${IGNITION_MSGS_INCLUDE_DIRS}
)

target_link_directories(
  ground_truth_node
  PRIVATE
  ${IGNITION_TRANSPORT_LIBRARY_DIRS}
  ${IGNITION_MSGS_LIBRARY_DIRS}
)

target_link_libraries(
  ground_truth_node
  ${IGNITION_TRANSPORT_LIBRARIES}
  ${IGNITION_MSGS_LIBRARIES}
)

ament_target_dependencies(
  ground_truth_node
  rclcpp
  geometry_msgs
)

install(
  TARGETS ground_truth_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS
  scripts/localization_benchmark.py
  scripts/square_test.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
