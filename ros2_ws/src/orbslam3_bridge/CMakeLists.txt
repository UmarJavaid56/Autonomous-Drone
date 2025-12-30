cmake_minimum_required(VERSION 3.8)
project(orbslam3_bridge)

# ROS2 / ament
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# OpenGL
find_package(OpenGL REQUIRED)

# Pangolin (installed in /usr/local by default)
# Adjust this if your Pangolin is somewhere else
set(Pangolin_DIR "/usr/local/lib/cmake/Pangolin")
find_package(Pangolin REQUIRED)

# ORB_SLAM3 root (installed externally)
set(ORB_SLAM3_ROOT "$ENV{HOME}/ORB_SLAM3")

include_directories(
  ${ORB_SLAM3_ROOT}                          # ORB_SLAM3 root
  ${ORB_SLAM3_ROOT}/include                  # for System.h, KeyFrame.h, etc.
  ${ORB_SLAM3_ROOT}/include/CameraModels     # <-- add this line
  ${ORB_SLAM3_ROOT}/Thirdparty/Sophus        # Sophus headers
  ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2         # DBoW2 headers
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)




# ORB_SLAM3 library directory
link_directories(${ORB_SLAM3_ROOT}/lib)

add_executable(orbslam3_mono_node
  src/orbslam3_mono_node.cpp
)

ament_target_dependencies(orbslam3_mono_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# If Pangolin exports definitions, propagate them
if(Pangolin_DEFINITIONS)
  target_compile_definitions(orbslam3_mono_node PRIVATE ${Pangolin_DEFINITIONS})
endif()

target_link_libraries(orbslam3_mono_node
  ORB_SLAM3                   # libORB_SLAM3.so from ORB_SLAM3/lib
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  ${OPENGL_LIBRARIES}
)

# Set rpath to find ORB_SLAM3 library at runtime
set_target_properties(orbslam3_mono_node PROPERTIES
  INSTALL_RPATH "$ENV{HOME}/ORB_SLAM3/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# Silence Eigen / g2o deprecation warnings
target_compile_definitions(orbslam3_mono_node PRIVATE EIGEN_NO_DEPRECATED_WARNING)
target_compile_options(orbslam3_mono_node PRIVATE -Wno-deprecated-declarations)


install(TARGETS
  orbslam3_mono_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

