cmake_minimum_required(VERSION 3.8)
project(esdf_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(esdf_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(esdf_server_node src/esdf_server_node.cpp)
ament_target_dependencies(esdf_server_node
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  geometry_msgs
  esdf_msgs
  visualization_msgs
  pcl_conversions
)
target_link_libraries(esdf_server_node ${PCL_LIBRARIES})

add_executable(point_cloud_filter_node src/point_cloud_filter_node.cpp)
ament_target_dependencies(point_cloud_filter_node
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_link_libraries(point_cloud_filter_node ${PCL_LIBRARIES})

install(TARGETS esdf_server_node point_cloud_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

ament_package()
