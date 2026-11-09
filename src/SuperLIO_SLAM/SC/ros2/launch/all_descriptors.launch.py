from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    std_config = os.path.join(get_package_share_directory('std_detector'),
                              'config', 'std_extract.yaml')
    btc_config = os.path.join(get_package_share_directory('btc_desc'),
                              'config', 'config_outdoor.yaml')
    topic = LaunchConfiguration('input_topic')
    root = LaunchConfiguration('output_root')
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/points_raw'),
        DeclareLaunchArgument('output_root', default_value='/tmp/descriptors'),
        DeclareLaunchArgument('btc_config', default_value=btc_config),
        DeclareLaunchArgument('max_visualized_descriptors', default_value='100'),
        Node(package='std_detector', executable='std_extract_node',
             name='std_extract_node', output='screen',
             parameters=[std_config, {'input_topic': topic,
                                      'output_dir': PathJoinSubstitution([root, 'std']),
                                      'max_visualized_descriptors': ParameterValue(
                                          LaunchConfiguration('max_visualized_descriptors'), value_type=int)}]),
        Node(package='btc_desc', executable='btc_extract_node',
             name='btc_extract_node', output='screen',
             parameters=[{'input_topic': topic,
                          'output_dir': PathJoinSubstitution([root, 'btc']),
                          'config_file': LaunchConfiguration('btc_config'),
                          'max_visualized_descriptors': ParameterValue(
                              LaunchConfiguration('max_visualized_descriptors'), value_type=int)}]),
        Node(package='scancontext_ros2', executable='scancontext_extract_node',
             name='scancontext_extract_node', output='screen',
             parameters=[{'input_topic': topic,
                          'output_dir': PathJoinSubstitution([root, 'scancontext'])}]),
    ])
