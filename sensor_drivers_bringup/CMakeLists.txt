################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(sensor_drivers_bringup)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(imu_fuse SHARED
  src/imu_fuse.cpp
)

rclcpp_components_register_node(imu_fuse
  PLUGIN "ImuFuse"
  EXECUTABLE imu_fuse_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch config
)

