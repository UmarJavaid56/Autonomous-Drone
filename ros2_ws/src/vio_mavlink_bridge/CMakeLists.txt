cmake_minimum_required(VERSION 3.5)
project(vio_mavlink_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(vio_mavlink_bridge_node
  src/vio_mavlink_bridge_node.cpp
)

add_executable(vio_sitl_test_node
 src/vio_sitl_test_node.cpp
 )

ament_target_dependencies(vio_mavlink_bridge_node
  rclcpp
  geometry_msgs
)

ament_target_dependencies(vio_sitl_test_node
 rclcpp
  geometry_msgs
  )

install(TARGETS
  vio_mavlink_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  vio_sitl_test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

