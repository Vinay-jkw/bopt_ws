cmake_minimum_required(VERSION 3.8)

project(ppts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==========================================================
# Dependencies
# ==========================================================

find_package(ament_cmake REQUIRED)

# ==========================================================
# Install executable Python files
# ==========================================================

install(
  PROGRAMS
  ppts_node_once.py
  ppts_visualize_cluster.py
  DESTINATION lib/${PROJECT_NAME}
)

# ==========================================================
# Install Python modules
# ==========================================================

install(
  DIRECTORY
  algorithms
  config
  detectors
  processors
  models
  transforms
  utils
  visualization
  DESTINATION lib/${PROJECT_NAME}
)

# ==========================================================
# Testing
# ==========================================================

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

endif()

# ==========================================================
# Package
# ==========================================================

ament_package()
