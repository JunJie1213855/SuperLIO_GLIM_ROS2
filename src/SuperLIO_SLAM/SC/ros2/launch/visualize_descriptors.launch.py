from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('scancontext_ros2')
    all_launch = os.path.join(share, 'launch', 'all_descriptors.launch.py')
    rviz_config = os.path.join(share, 'rviz', 'descriptor_visualization.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/points_raw'),
        DeclareLaunchArgument('output_root', default_value='/tmp/descriptors'),
        DeclareLaunchArgument('max_visualized_descriptors', default_value='100'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(all_launch),
            launch_arguments={
                'input_topic': LaunchConfiguration('input_topic'),
                'output_root': LaunchConfiguration('output_root'),
                'max_visualized_descriptors': LaunchConfiguration('max_visualized_descriptors'),
            }.items()),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', LaunchConfiguration('rviz_config')]),
    ])
