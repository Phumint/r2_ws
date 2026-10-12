"""Empty gz sim world, then spawn the field URDF (static) and, optionally, every game object URDF.
Also publishes the field on /field_description for RViz.

    ros2 launch robocon2027_field field_urdf.launch.py            # field + objects
    ros2 launch robocon2027_field field_urdf.launch.py objects:=false
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robocon2027_field')
    field_urdf = os.path.join(pkg, 'urdf', 'robocon2027_field.urdf')
    with open(field_urdf) as f:
        field_xml = f.read()
    objects = json.load(open(os.path.join(pkg, 'config', 'objects.json')))
    with_objects = LaunchConfiguration('objects')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('objects', default_value='true',
                                        description='spawn the 52 blocks and the Mustika'))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()))
    ld.add_action(Node(package='robot_state_publisher', executable='robot_state_publisher',
                       name='field_state_publisher',
                       parameters=[{'robot_description': field_xml}],
                       remappings=[('robot_description', 'field_description')]))
    ld.add_action(Node(package='ros_gz_sim', executable='create', output='screen',
                       arguments=['-name', 'robocon2027_field', '-file', field_urdf,
                                  '-x', '0', '-y', '0', '-z', '0']))
    for o in objects:
        x, y, z = o['xyz']
        ld.add_action(Node(package='ros_gz_sim', executable='create', output='log',
                           condition=IfCondition(with_objects),
                           arguments=['-name', o['name'], '-file', os.path.join(pkg, 'urdf', 'objects', o['urdf']),
                                      '-x', str(x), '-y', str(y), '-z', str(z)]))
    return ld
