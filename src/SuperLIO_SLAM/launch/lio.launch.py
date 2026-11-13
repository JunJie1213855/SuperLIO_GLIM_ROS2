"""只启动 LIO 前端；关键帧发布、回环后端和显示分别配置。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _lio_node(context):
    overrides = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'slam.keyframe.enable': ParameterValue(LaunchConfiguration('publish_keyframes'), value_type=bool),
    }
    mode = LaunchConfiguration('downsample_mode').perform(context)
    if mode:
        overrides['lio.downsample.mode'] = mode
    return [Node(package='super_lio_slam', executable='super_lio_node', name='super_lio_node',
                 namespace='', output='screen',
                 parameters=[LaunchConfiguration('sensor_config'), LaunchConfiguration('slam_config'), overrides],
                 arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')])]


def generate_launch_description():
    share = FindPackageShare('super_lio_slam')
    config = lambda name: PathJoinSubstitution([share, 'config', name])
    return LaunchDescription([
        DeclareLaunchArgument('sensor_config', default_value=config('common/robosense_airy.yaml')),
        DeclareLaunchArgument('slam_config', default_value=config('loop/slam.yaml'),
                              description='Only the super_lio_node keyframe settings are read'),
        DeclareLaunchArgument('publish_keyframes', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('downsample_mode', default_value='', choices=['', 'fixed', 'adaptive'],
                              description='LIO sampling; empty uses sensor YAML (default fixed)'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_lio_node),
    ])
