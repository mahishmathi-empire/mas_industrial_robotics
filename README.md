## Direct Base Controller

### Description
Given a taget pose the youbot should succesfully reach that pose with its movement i.e. Its linear and its angular velocites synchronized. The code is written in C++ language with node lifecycle implemmented.

## Prerequisite
1. Install cross to with gazebo.
2. Installs CPP compiler.
3. Install and IDE.

## Installation

## Usage
1. Launch gazebo and spawn the youbot in the environment.
   
   ```
    ros2 launch youbot_gazebo start_environment.py
   ```
   ```
    ros2 launch youbot_gazebo spawn_robot.xml
   ```
2. Run the ROS node
   ```
    ros2 launch mir_direct
   ```
   
4. Launch RQT to publish target pose
   ```
    rqt
   ```



## Configuration
Set configuration file as follows

Change launch file as follows  


Steps to finalizew software
1. implement obstacle avoidance functionality
2. set dynamic configauration functionality
3. implement ros node lifecycle
4. Test it in Youbot

Features of the prototype
1.Given a target pose the 
