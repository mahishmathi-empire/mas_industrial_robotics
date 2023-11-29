cmake_minimum_required(VERSION 3.8)
project(mir_pick_object)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mir_interfaces REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp
  std_msgs
  mir_interfaces
)

include_directories(
  include
)

add_executable(pick_object
  src/pick_object.cpp
  src/pick_object_test.cpp
)

ament_target_dependencies(pick_object
  ${dependencies}
)

install(TARGETS
  pick_object
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
 bt/
 DESTINATION share/${PROJECT_NAME}/bt
)

ament_export_dependencies(${dependencies})

ament_package()
