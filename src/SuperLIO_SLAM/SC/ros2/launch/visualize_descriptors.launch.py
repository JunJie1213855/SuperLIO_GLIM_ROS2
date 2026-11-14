from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('scancontext_ros2')
    extract_launch = os.path.join(share, 'launch', 'scancontext_extract.launch.py')
    rviz_config = os.path.join(share, 'rviz', 'descriptor_visualization.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/points_raw'),
        DeclareLaunchArgument('output_dir', default_value='/tmp/scancontext_descriptors'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(extract_launch),
            launch_arguments={
                'input_topic': LaunchConfiguration('input_topic'),
                'output_dir': LaunchConfiguration('output_dir'),
            }.items()),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', LaunchConfiguration('rviz_config')]),
    ])
