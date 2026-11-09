cmake_minimum_required(VERSION 3.8)
project(scancontext_ros2)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
add_library(scancontext_core cpp/module/Scancontext/Scancontext.cpp)
target_include_directories(scancontext_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/module/Scancontext>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(scancontext_core ${PCL_LIBRARIES})
add_executable(scancontext_extract_node ros2/scancontext_extract_node.cpp)
ament_target_dependencies(scancontext_extract_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(scancontext_extract_node scancontext_core ${PCL_LIBRARIES})
install(TARGETS scancontext_extract_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS scancontext_core EXPORT export_scancontext_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY cpp/module/Scancontext/ DESTINATION include FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
install(FILES ros2/launch/scancontext_extract.launch.py
              ros2/launch/all_descriptors.launch.py
              ros2/launch/visualize_descriptors.launch.py
        DESTINATION share/${PROJECT_NAME}/launch)
install(FILES ros2/rviz/descriptor_visualization.rviz
        DESTINATION share/${PROJECT_NAME}/rviz)
ament_export_targets(export_scancontext_core HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs pcl_conversions PCL)
ament_package()
