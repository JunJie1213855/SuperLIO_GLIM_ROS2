"""Shared SuperLIO mapping, optional loop backend, and RViz entry point."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _make_loop_backend(context):
    # Use the ABI selected at build time even if the parent terminal sources ROS last.
    share = Path(FindPackageShare('super_lio_slam').perform(context))
    runtime_file = share / 'config' / 'runtime_library_dirs.txt'
    if not runtime_file.is_file():
        raise RuntimeError('Missing runtime library configuration; rebuild super_lio_slam')
    directories = [line.strip() for line in runtime_file.read_text().splitlines() if line.strip()]
    if not directories or any(not Path(directory).is_dir() for directory in directories):
        raise RuntimeError('Build-time runtime libraries moved; rebuild super_lio_slam')
    inherited = context.environment.get('LD_LIBRARY_PATH', '').split(os.pathsep)
    library_path = os.pathsep.join(dict.fromkeys(directory for directory in [*directories, *inherited] if directory))
    save_overrides = {}
    for name, value_type in [('save_map_path', str), ('save_map_voxel_size', float),
                             ('save_map_on_shutdown', bool)]:
        if LaunchConfiguration(name).perform(context):
            save_overrides[name] = ParameterValue(LaunchConfiguration(name), value_type=value_type)
    return [Node(
        package='super_lio_slam', executable='loop_backend_node', name='loop_backend_node', namespace='',
        output='screen', additional_env={'LD_LIBRARY_PATH': library_path},
        parameters=[LaunchConfiguration('slam_config'), LaunchConfiguration('registration_config'), {
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'loop_registration_mode': LaunchConfiguration('loop_registration_mode'),
            'loop_candidate_mode': LaunchConfiguration('loop_candidate_mode'),
            'loop_candidate_radius_m': ParameterValue(LaunchConfiguration('loop_candidate_radius_m'), value_type=float),
            **save_overrides,
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        sigterm_timeout='120', sigkill_timeout='120',
    )]


def generate_launch_description():
    share = FindPackageShare('super_lio_slam')
    config = lambda name: PathJoinSubstitution([share, 'config', name])
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    enable_loop = ParameterValue(LaunchConfiguration('enable_loop_backend'), value_type=bool)
    default_rviz = PathJoinSubstitution([
        share, 'rviz', PythonExpression([
            "'slam.rviz' if '", LaunchConfiguration('enable_loop_backend'),
            "' == 'true' else 'lio.rviz'",
        ]),
    ])
    return LaunchDescription([
        DeclareLaunchArgument('sensor_config', default_value=config('livox_360.yaml'),
                              description='Sensor and SuperLIO front-end YAML'),
        DeclareLaunchArgument('slam_config', default_value=config('slam.yaml'),
                              description='Keyframe, STD, iSAM2 and map parameters'),
        DeclareLaunchArgument('registration_config', default_value=config('loop_teaser_gicp.yaml'),
                              description='Shared GICP quality and optional TEASER++ parameters'),
        DeclareLaunchArgument('enable_loop_backend', default_value='true', choices=['true', 'false'],
                              description='Enable keyframes and the online pose-graph backend'),
        DeclareLaunchArgument('loop_candidate_mode', default_value='std',
                              choices=['distance', 'std'], description='Loop candidate retrieval'),
        DeclareLaunchArgument('loop_candidate_radius_m', default_value='15.0',
                              description='Maximum raw-odometry candidate distance in both retrieval modes'),
        DeclareLaunchArgument('loop_registration_mode', default_value='std_teaser_gicp',
                              choices=['std_gicp', 'std_teaser_gicp'],
                              description='Loop registration algorithm'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('save_map_path', default_value='',
                              description='Optimized PCD output path; empty uses slam_config'),
        DeclareLaunchArgument('save_map_voxel_size', default_value='',
                              description='Saved map voxel size in metres; 0 disables fusion; empty uses YAML'),
        DeclareLaunchArgument('save_map_on_shutdown', default_value='', choices=['', 'true', 'false'],
                              description='Save optimized map on graceful exit; empty uses YAML'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz,
                              description='RViz file; automatically matches backend/front-end mode'),
        DeclareLaunchArgument('log_level', default_value='info',
                              choices=['debug', 'info', 'warn', 'error', 'fatal']),
        DeclareLaunchArgument('rviz_log_level', default_value='warn',
                              choices=['debug', 'info', 'warn', 'error', 'fatal']),
        # Explicit namespaces make inline overrides node-specific rather than wildcard YAML.
        Node(
            package='super_lio_slam', executable='super_lio_node', name='super_lio_node', namespace='',
            output='screen',
            parameters=[LaunchConfiguration('sensor_config'), LaunchConfiguration('slam_config'), {
                'use_sim_time': use_sim_time,
                'slam.keyframe.enable': enable_loop,
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
        OpaqueFunction(function=_make_loop_backend,
                       condition=IfCondition(LaunchConfiguration('enable_loop_backend'))),
        Node(
            package='rviz2', executable='rviz2', name='super_lio_slam_rviz', namespace='', output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', LaunchConfiguration('rviz_config'), '--ros-args', '--log-level',
                       LaunchConfiguration('rviz_log_level')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
