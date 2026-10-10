"""Cameras only for native LeRobot recording; never opens robot serial ports."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('so101_bringup'),'launch','include','camera.launch.py')))])
