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


add_executable(direct_base_controller 
  src/main.cpp
  src/component_wise_pose_error_calculator.cpp
  src/twist_limiter.cpp
)

target_include_directories(direct_base_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(direct_base_controller 
rclcpp 
geometry_msgs
tf2
)

install(TARGETS
  direct_base_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
