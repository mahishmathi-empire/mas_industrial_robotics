# mas_industrial_robotics - humble/bringup

The branch contains the necessary packages to bringup the robot with driver controllers.

## Dependent packages
- control_msgs
- urg_node
- joy_node
- moveit_msgs
- xacro
- twist-mux
- joint-state-publisher
- joint-state-publisher-gui
- realsense sdk and realsense2-ros

## Setup

- Create a workspace
```bash
mkdir -p ~/mir/src
```

- Clone the humble/bringup branch.
```bash
cd ~/mir/src

git clone -b humble https://github.com/mas_industrial_robotics.git
```

- Clone the dependency packages
```
cd ~/mir/src

vcs import < mas_industrial_robots/mir.repos
```

- Launch the bringup
```bash
ros2 launch mir_bringup robot.launch.py
```

## Direct Base Controller

## Description

By giving a target pose, the youbot should successfully move towards the pose. For addressing the problem, the linear and angular velocities must be synchronized. The code is written in C++ language with node lifecycle implemented.

## Features impelemented

1. Node Life Cycle
2. Dynamic Variable Configuration
3. Synchronized linear, angular motion
4. Obstacle detection

## Environmental setup

1. Install ROS rolling. And then source it.

```
sudo apt install ros-rolling-desktop
source /opt/ros/rolling/setup.bash
```

2. Install gazebo
```
sudo apt-get install ros-rolling-ros-gz
```
3. Install vscode
For more information please refer to [vscode download page](https://code.visualstudio.com/docs/setup/linux).


## Packages
The necessary packages are as following:
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

* To understand execution details, please refer to [wikipage](https://github.com/HBRS-SDP/ws23-direct-base-controller/wiki).