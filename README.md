# mas_industrial_robotics

## Status

### Working modules

- mir_bringup
- mir_perception
  - mir_object_recognition
  - mir_ppt_detection

### Modules under development

- mir_navigation
- mir_manipulation
- mir_actions


## Setup

- Create a workspace
  ```bash
  mkdir -p ~/mir/src
  ```

- Clone the rolling branch.
  ```bash
  cd ~/mir/src

  git clone -b rolling https://github.com/mas_industrial_robotics.git
  ```

- Clone the dependency packages
  ```bash
  cd ~/mir/src

  vcs import < mas_industrial_robotics/mir.repos
  ```

- Build the workspace
  ```bash
  cd ~/mir

  colcon build --symlink-install
  ```

### Usage

  - View the youbot in rviz
    ```bash
    ros2 launch mir_bringup view_youbot.launch.py
    ```
