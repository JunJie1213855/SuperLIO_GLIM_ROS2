"""只启动 RViz；支持 LIO / 回环 / 重定位预设和外部 RViz 配置。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _rviz(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    config = value('rviz_config') or os.path.join(
        get_package_share_directory('super_lio_slam'), 'rviz', value('profile') + '.rviz')
    return [Node(package='rviz2', executable='rviz2', name='super_lio_rviz', output='screen',
                 arguments=['-d', config, '--ros-args', '--log-level', value('log_level')],
                 parameters=[{'use_sim_time': value('use_sim_time') == 'true'}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='lio', choices=['lio', 'slam', 'relocation']),
        DeclareLaunchArgument('rviz_config', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('log_level', default_value='warn'),
        OpaqueFunction(function=_rviz),
    ])
