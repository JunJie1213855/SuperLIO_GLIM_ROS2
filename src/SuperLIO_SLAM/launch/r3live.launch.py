"""Mapping profile for r3live.yaml; all backend options come from slam.launch.py."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('super_lio_slam')
    return LaunchDescription([
        DeclareLaunchArgument('sensor_config', default_value=PathJoinSubstitution(
            [share, 'config', 'common/r3live.yaml'])),
        DeclareLaunchArgument('slam_config', default_value=PathJoinSubstitution(
            [share, 'config', 'loop/slam.yaml'])),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'slam.launch.py'])),
            launch_arguments={
                'sensor_config': LaunchConfiguration('sensor_config'),
                'slam_config': LaunchConfiguration('slam_config'),
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
    ])
