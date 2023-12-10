## Direct Base Controller

### Description
Given a target pose the youbot should successfully reach that pose with its movement i.e. Its linear and its angular velocities synchronized. The code is written in C++ language with node lifecycle implemented.

## Prerequisite
1. Install cross to with gazebo.
2. Installs CPP compiler.
3. Install and IDE.

## Installation

## Usage
1. Launch gazebo and spawn the youbot in the environment.
   
   ```
    ros2 launch youbot_gazebo start_world.launch.py
   ```
   ```
    ros2 launch youbot_gazebo spawn_youbot_ros2.launch.xml
   ```
2. Run the ROS node
   ```
    ros2 launch mir_direct_base_controller direct_base_controller.launch.py
   ```
   
4. Launch RQT to publish target pose
   ```
    rqt
   ```
   ![rqt1](https://github.com/HBRS-SDP/ws23-direct-base-controller/assets/71880369/8e88f344-f157-4502-86e0-8214145c41b1)



## Configuration
Set configuration file as follows

Change launch file as follows  


