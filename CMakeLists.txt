cmake_minimum_required(VERSION 3.8)
project(mujoco_ros2_interface LANGUAGES CXX)

# ==========================================================
# Compiler setup
# ==========================================================
# Add useful compiler warnings for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Require C++17 or newer (MuJoCo 3.x requires at least this)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ==========================================================
# Dependencies
# ==========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

# ==========================================================
# Include and library directories
# ==========================================================
# Expect MUJOCO_HOME environment variable (e.g., /home/user/mujoco-3.3.6)
if(NOT DEFINED ENV{MUJOCO_HOME})
  message(FATAL_ERROR "Environment variable MUJOCO_HOME is not set. Please export it to your MuJoCo installation path.")
endif()

include_directories(
  include
  $ENV{MUJOCO_HOME}/include
)

link_directories(
  $ENV{MUJOCO_HOME}/lib
)

# ==========================================================
# Executable definition
# ==========================================================
add_executable(mujoco_node
  src/main.cpp
  src/mujoco_node.cpp
  src/mujoco_simulator.cpp
  src/mujoco_visualizer.cpp
)

# ==========================================================
# Link libraries and dependencies
# ==========================================================
# MuJoCo, GLFW, math, and threading
target_link_libraries(mujoco_node
  mujoco
  glfw
  Threads::Threads
  m
)

# ROS 2 dependencies
ament_target_dependencies(mujoco_node
  rclcpp
  sensor_msgs
  std_msgs
)

# ==========================================================
# Installation rules
# ==========================================================
install(TARGETS mujoco_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch mjcf
  DESTINATION share/${PROJECT_NAME}
)

# ==========================================================
# Testing (optional)
# ==========================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ==========================================================
# Package export
# ==========================================================
ament_package()
