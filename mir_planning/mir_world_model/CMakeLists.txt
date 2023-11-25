cmake_minimum_required(VERSION 3.8)
project(mir_world_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mir_interfaces REQUIRED)

include_directories(
  include
)

add_library(world_model SHARED
  src/world_model.cpp
)

add_executable(world_model_node
  src/world_model_node.cpp
)

ament_target_dependencies(world_model_node
  rclcpp
  mir_interfaces
  world_model
)

install(TARGETS
  world_model_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
 config/
 DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
