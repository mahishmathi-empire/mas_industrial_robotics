## Direct Base Controller

### Description
Given a target pose the youbot should successfully move toward the pose. For addressing the problem, the linear and angular velocities must be synchronized. The code totally is written in C++ language with node lifecycle implemented.

## Prerequisite
1. Ros 2, robile version.
2. gazebo.
3. C++ compiler.
4. An IDE supporing Ros 2 and c++, preferably vscode.

## Installation ()
Packages required are as following:
1. rclcpp
2. geometry_msgs
3. tf2
4. tf2_ros
5. tf2_geometry_msgs
6. tf2_sensor_msgs
7. sensor_msgs
8. rclcpp_lifecycle

## Execution
1. Launch gazebo and spawn the youbot in the environment.
   
   ```
    ros2 launch youbot_gazebo start_world.launch.py
   ```
   ```
    ros2 launch youbot_gazebo spawn_youbot_ros2.launch.xml
   ```
2. Run the ROS node.
   ```
    ros2 launch mir_direct_base_controller direct_base_controller.launch.py
   ```
   
3. Launch RQT to publish target pose.
   ```
    rqt
   ```
   ![rqt1](https://github.com/HBRS-SDP/ws23-direct-base-controller/assets/71880369/8e88f344-f157-4502-86e0-8214145c41b1)

4. Run the lifecycle node.

   ```
   ros2 run lifecycle_controller lifecycle_controller  --ros-args -p lc_name:=direct_base_controller
   ```