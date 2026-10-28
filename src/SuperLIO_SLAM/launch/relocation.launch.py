"""Motion-compensated local LIO -> TEASER++ / GICP -> world-map tracking."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _relocation_node(context):
    share=Path(FindPackageShare('super_lio_slam').perform(context))
    runtime=share/'config'/'runtime_library_dirs.txt'
    if not runtime.is_file():
        raise RuntimeError('Missing runtime library configuration; rebuild super_lio_slam')
    directories=[line.strip() for line in runtime.read_text().splitlines() if line.strip()]
    if not directories or any(not Path(path).is_dir() for path in directories):
        raise RuntimeError('Build-time libraries moved; rebuild super_lio_slam')
    inherited=context.environment.get('LD_LIBRARY_PATH','').split(os.pathsep)
    library_path=os.pathsep.join(dict.fromkeys(path for path in [*directories,*inherited] if path))
    overrides={'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}
    map_path=LaunchConfiguration('map_path').perform(context)
    if map_path:
        overrides['relocation.map_path']=map_path
    return [Node(package='super_lio_slam',executable='relocation_node',name='relocation_node',namespace='',
                 output='screen',parameters=[LaunchConfiguration('sensor_config'),
                 LaunchConfiguration('relocation_config'),overrides],additional_env={'LD_LIBRARY_PATH':library_path},
                 arguments=['--ros-args','--log-level',LaunchConfiguration('log_level')])]


def generate_launch_description():
    share=FindPackageShare('super_lio_slam')
    config=lambda name: PathJoinSubstitution([share,'config',name])
    return LaunchDescription([
        DeclareLaunchArgument('sensor_config',default_value=config('robosense_airy.yaml')),
        DeclareLaunchArgument('relocation_config',default_value=config('relocation.yaml')),
        DeclareLaunchArgument('map_path',default_value='',description='Absolute reference PCD path; empty uses YAML'),
        DeclareLaunchArgument('use_sim_time',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('rviz',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('rviz_config',default_value=PathJoinSubstitution([share,'rviz','relocation.rviz'])),
        DeclareLaunchArgument('log_level',default_value='info'),
        DeclareLaunchArgument('rviz_log_level',default_value='warn'),
        OpaqueFunction(function=_relocation_node),
        Node(package='rviz2',executable='rviz2',name='super_lio_slam_rviz',namespace='',output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d',LaunchConfiguration('rviz_config'),'--ros-args','--log-level',LaunchConfiguration('rviz_log_level')],
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}]),
    ])
