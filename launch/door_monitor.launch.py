from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default=Path(get_package_share_directory('door_monitor'))/'config'/'door.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('params_file',default_value=str(default)),
        Node(package='door_monitor',executable='door_monitor',name='door_monitor',
             parameters=[LaunchConfiguration('params_file')],output='screen')])
