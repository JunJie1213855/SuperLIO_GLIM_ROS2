"""只启动在线回环 / iSAM2 / 优化地图保存后端，输入为 LIO 关键帧。"""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _backend(context):
    share = Path(FindPackageShare('super_lio_slam').perform(context))
    runtime_file = share / 'config' / 'runtime_library_dirs.txt'
    if not runtime_file.is_file():
        raise RuntimeError('Missing runtime library configuration; rebuild super_lio_slam')
    directories = [line.strip() for line in runtime_file.read_text().splitlines() if line.strip()]
    if not directories or any(not Path(path).is_dir() for path in directories):
        raise RuntimeError('Build-time runtime libraries moved; rebuild super_lio_slam')
    inherited = context.environment.get('LD_LIBRARY_PATH', '').split(os.pathsep)
    library_path = os.pathsep.join(dict.fromkeys(path for path in [*directories, *inherited] if path))
    overrides = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    # 空参数保留 YAML 的配置，只有命令行显式指定才覆盖。
    optional = [('use_teaserpp', bool), ('loop_registration_mode', str), ('loop_candidate_mode', str),
                ('loop_candidate_radius_m', float), ('save_map_path', str),
                ('save_map_voxel_size', float), ('save_map_on_shutdown', bool)]
    for name, value_type in optional:
        if LaunchConfiguration(name).perform(context):
            overrides[name] = ParameterValue(LaunchConfiguration(name), value_type=value_type)
    return [Node(
        package='super_lio_slam', executable='loop_backend_node', name='loop_backend_node',
        namespace='', output='screen', additional_env={'LD_LIBRARY_PATH': library_path},
        parameters=[LaunchConfiguration('slam_config'), LaunchConfiguration('registration_config'), overrides],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        sigterm_timeout='120', sigkill_timeout='120',
    )]


def generate_launch_description():
    share = FindPackageShare('super_lio_slam')
    config = lambda name: PathJoinSubstitution([share, 'config', name])
    arguments = [
        DeclareLaunchArgument('slam_config', default_value=config('loop/slam.yaml')),
        DeclareLaunchArgument('registration_config', default_value=config('loop/loop_teaser_gicp.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('use_teaserpp', default_value='', choices=['', 'true', 'false'],
                              description='Loop coarse registration; empty uses YAML or registration mode default'),
    ]
    for name in ['loop_registration_mode', 'loop_candidate_mode', 'loop_candidate_radius_m',
                 'save_map_path', 'save_map_voxel_size', 'save_map_on_shutdown']:
        arguments.append(DeclareLaunchArgument(name, default_value='', description='Empty uses YAML'))
    return LaunchDescription([*arguments, OpaqueFunction(function=_backend)])
