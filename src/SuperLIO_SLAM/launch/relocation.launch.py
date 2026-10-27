"""Run map-based relocation with configurable sensor parameters and ROS time."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('super_lio_slam')
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('sensor_config', default_value=PathJoinSubstitution(
            [share, 'config', 'robosense_airy.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution(
            [share, 'rviz', 'relocation.rviz'])),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('rviz_log_level', default_value='warn'),
        Node(
            package='super_lio_slam', executable='relocation_node', name='relocation_node', namespace='',
            output='screen', parameters=[LaunchConfiguration('sensor_config'), {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
        Node(
            package='rviz2', executable='rviz2', name='super_lio_slam_rviz', namespace='', output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', LaunchConfiguration('rviz_config'), '--ros-args', '--log-level',
                       LaunchConfiguration('rviz_log_level')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
