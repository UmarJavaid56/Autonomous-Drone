cmake_minimum_required(VERSION 3.10)
project(vio_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/camera_bridge.py
  scripts/camera_simulator.py
  scripts/gz_imu_bridge.py
  scripts/gz_physics_imu_bridge.py
  scripts/imu_simulator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

