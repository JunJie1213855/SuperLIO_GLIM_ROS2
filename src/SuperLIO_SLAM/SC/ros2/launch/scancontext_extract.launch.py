from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/points_raw'),
        DeclareLaunchArgument('output_dir', default_value='/tmp/scancontext_descriptors'),
        Node(package='scancontext_ros2', executable='scancontext_extract_node',
             name='scancontext_extract_node', output='screen',
             parameters=[{'input_topic': LaunchConfiguration('input_topic'),
                          'output_dir': LaunchConfiguration('output_dir')}]),
    ])
