from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
import os

def generate_launch_description():
    params = os.path.join(get_package_share_directory('mir_direct_base_controller'), 'config','config.yaml')
    # add the gripper controller node
    DirectBaseController = Node(
        package='mir_direct_base_controller',
        executable='direct_base_controller',
        name='direct_base_controller',
        output='both',
        parameters=[params],
        remappings=[('cmd_vel', '/cmd_vel'), ('target_pose', '/target_pose')]
    )

    return LaunchDescription([
        DirectBaseController,
    ])