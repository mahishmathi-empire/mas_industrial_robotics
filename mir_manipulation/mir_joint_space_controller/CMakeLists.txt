cmake_minimum_required(VERSION 2.8.3)
project(mir_joint_space_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    tf
)

catkin_package()

include_directories(
    ros/include
    ${catkin_INCLUDE_DIRS}
)

add_library(joint_space_controller
    SHARED
        ros/src/joint_space_controller.cpp
        ros/src/utils.cpp
)
target_link_libraries(joint_space_controller
    ${catkin_LIBRARIES}
)

# ==========
# EXECUTABLE
# ==========
add_executable(joint_space_controller_node
    ros/src/main.cpp
)
target_link_libraries(joint_space_controller_node
    ${catkin_LIBRARIES}
    joint_space_controller
)

