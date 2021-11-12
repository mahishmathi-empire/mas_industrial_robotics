cmake_minimum_required(VERSION 2.8.3)
project(mir_joint_space_controller)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    visualization_msgs
    tf
    mir_pregrasp_planning
)
