cmake_minimum_required(VERSION 3.12)
project(mir_world_model)

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
find_package(mir_interfaces REQUIRED)

include_directories(
  include
)

add_library(world_model SHARED
  src/world_model.cpp
)

ament_target_dependencies(world_model
  rclcpp
  mir_interfaces
)

add_executable(world_model_node
  src/world_model_node.cpp
)

ament_target_dependencies(world_model_node
  rclcpp
  mir_interfaces
)

target_link_libraries(world_model_node
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
