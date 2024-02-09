cmake_minimum_required(VERSION 3.5)

project(mir_direct_base_controller)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# find_package(laser_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL 1.10 REQUIRED)

add_executable(direct_base_controller 
  # src/main.cpp
  src/component_wise_pose_error_calculator.cpp
  src/twistsynchronizer.cpp
  src/twist_synchronizer_utils.cpp
  src/twist_limiter.cpp
  src/twist_controller.cpp
  src/component_wise_pose_error_monitor.cpp
  src/direct_base_controller_coordinator.cpp
  # src/laser_distances_node.cpp
)

target_include_directories(direct_base_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})

ament_target_dependencies(direct_base_controller 
rclcpp 
geometry_msgs
tf2
tf2_geometry_msgs
tf2_sensor_msgs
tf2_ros
sensor_msgs
pcl_conversions 
pcl_ros
)

install(TARGETS
  direct_base_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
