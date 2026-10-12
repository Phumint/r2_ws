"""Gazebo (gz sim) with the complete world: field + 40 Earth Blocks + 12 Sky Blocks + Mustika.

    ros2 launch robocon2027_field field.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('robocon2027_field')
    world = os.path.join(pkg, 'worlds', 'robocon2027.sdf')
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + world}.items())
    return LaunchDescription([gz])
