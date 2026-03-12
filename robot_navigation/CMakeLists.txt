# -------------------------------------------------------------
# Minimum CMake version required for this project
# -------------------------------------------------------------
cmake_minimum_required(VERSION 3.8)

# Name of the ROS2 package / project
project(robot_navigation)

# -------------------------------------------------------------
# Enable useful compiler warnings when using GCC or Clang
# -------------------------------------------------------------
# -Wall      : enable common warnings
# -Wextra    : enable extra warnings
# -Wpedantic : enforce strict ISO C++ compliance
# -------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------------
# Find required dependencies for this ROS2 package
# -------------------------------------------------------------
# ament_cmake      -> ROS2 build system
# rclcpp           -> ROS2 C++ client library
# geometry_msgs    -> velocity command messages
# nav_msgs         -> odometry and path messages
# tf2              -> quaternion and coordinate transformations
# -------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

# -------------------------------------------------------------
# Optional testing configuration
# -------------------------------------------------------------
# Enables ROS2 linting and static analysis tools during build
# when BUILD_TESTING is enabled.
# -------------------------------------------------------------
if(BUILD_TESTING)

  # Automatically find and run available linters
  find_package(ament_lint_auto REQUIRED)

  # Skip copyright check
  # (used when license headers are not yet added to files)
  set(ament_cmake_copyright_FOUND TRUE)

  # Skip cpplint when not inside a git repository
  set(ament_cmake_cpplint_FOUND TRUE)

  # Load all linting dependencies
  ament_lint_auto_find_test_dependencies()

endif()

# -------------------------------------------------------------
# Declare ROS2 package
# -------------------------------------------------------------
ament_package()

# -------------------------------------------------------------
# Build the controller executable
# -------------------------------------------------------------
# The controller node includes:
#
#   controller.cpp           -> PID trajectory tracking node
#   path_smoother.cpp        -> path smoothing algorithm
#   trajectory_generator.cpp -> trajectory generation logic
# -------------------------------------------------------------
add_executable(controller
  src/controller.cpp
  src/path_smoother.cpp
  src/trajectory_generator.cpp
)

# -------------------------------------------------------------
# Link ROS2 dependencies with the executable
# -------------------------------------------------------------
ament_target_dependencies(controller
rclcpp
geometry_msgs
nav_msgs
tf2)

# -------------------------------------------------------------
# Include project header files
# -------------------------------------------------------------
# This allows source files to access headers located in
# the "include" directory of the package.
# -------------------------------------------------------------
target_include_directories(controller PUBLIC
  include
)

# -------------------------------------------------------------
# Install compiled executable
# -------------------------------------------------------------
# Installs the controller node binary so it can be launched
# using ROS2 launch files.
# -------------------------------------------------------------
install(TARGETS
controller
DESTINATION lib/${PROJECT_NAME})

# -------------------------------------------------------------
# Install launch files and configuration files
# -------------------------------------------------------------
# These include:
#
#   launch/sim.launch.py
#   launch/navigation.rviz
#   config/controller_params.yaml
#
# Installing them allows ROS2 to locate these files using
# get_package_share_directory().
# -------------------------------------------------------------
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


