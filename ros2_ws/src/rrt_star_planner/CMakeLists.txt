cmake_minimum_required(VERSION 3.8)
project(rrt_star_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(esdf_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ompl REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})

add_executable(rrt_star_planner_node src/rrt_star_planner_node.cpp)
ament_target_dependencies(rrt_star_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  esdf_msgs
  std_msgs
)

target_link_libraries(rrt_star_planner_node
  ${OMPL_LIBRARIES}
)

install(TARGETS rrt_star_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
