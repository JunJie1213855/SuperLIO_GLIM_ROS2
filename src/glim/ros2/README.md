# GLIM ROS 2 接口

此目录由父级 `glim` 包统一构建，不再是独立 ROS 包。

```bash
colcon build --packages-select glim
source install/setup.bash
ros2 run glim glim_rosnode
ros2 run glim glim_rosbag <bag目录>
ros2 run glim offline_viewer <dump目录>
ros2 run glim map_editor <dump目录>
```

组件为 `glim::GlimROS`，组件资源归属 `glim`。头文件仍使用 `glim_ros/...`，运行库仍为 `libglim_ros.so` 和 `librviz_viewer.so`，原有话题、配置键不变。RViz 配置安装在 `share/glim/rviz`。非 ROS 构建不编译此目录。

详细流程见 [工作区 Quickstart](../../../docs/quickstart.md)。
