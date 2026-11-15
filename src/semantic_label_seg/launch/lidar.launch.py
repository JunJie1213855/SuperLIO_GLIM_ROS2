from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = str(
        Path(get_package_share_directory("semantic_label_seg")) / "config/lidar.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default),
            Node(
                package="semantic_label_seg",
                executable="lidar_seg_node",
                name="lidar_seg",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
