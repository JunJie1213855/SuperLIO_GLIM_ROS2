"""Run standalone STD extraction with typed parameters and configurable ROS time."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('std_config', default_value=PathJoinSubstitution(
            [FindPackageShare('super_lio_slam'), 'config', 'loop/std_extract.yaml'])),
        DeclareLaunchArgument('input_topic', default_value='/points_raw'),
        DeclareLaunchArgument('output_dir', default_value='/tmp/std_descriptors'),
        DeclareLaunchArgument('max_visualized_descriptors', default_value='100'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='super_lio_slam', executable='std_extract_node', name='std_extract_node', namespace='',
            output='screen', parameters=[LaunchConfiguration('std_config'), {
                'input_topic': ParameterValue(LaunchConfiguration('input_topic'), value_type=str),
                'output_dir': ParameterValue(LaunchConfiguration('output_dir'), value_type=str),
                'max_visualized_descriptors': ParameterValue(
                    LaunchConfiguration('max_visualized_descriptors'), value_type=int),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ])
