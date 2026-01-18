#!/bin/bash
# Install RTAB-Map dependencies for ROS 2 Jazzy

set -e

echo "Installing RTAB-Map ROS 2 packages..."

# Update package list
sudo apt update

# Install RTAB-Map and related packages
sudo apt install -y \
    ros-jazzy-rtabmap \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-rtabmap-odom \
    ros-jazzy-rtabmap-slam \
    ros-jazzy-rtabmap-util \
    ros-jazzy-rtabmap-viz \
    ros-jazzy-rtabmap-msgs \
    ros-jazzy-rtabmap-conversions \
    ros-jazzy-rtabmap-python

# Install additional dependencies for point cloud processing
sudo apt install -y \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    libpcl-dev

# Install visualization tools
sudo apt install -y \
    ros-jazzy-rviz2 \
    ros-jazzy-rviz-default-plugins \
    ros-jazzy-rviz-visual-tools

echo "RTAB-Map installation complete!"
echo ""
echo "To use RTAB-Map SLAM:"
echo "  ros2 launch rtabmap_slam x500_rtabmap_slam.launch.py"
echo ""
echo "For more information, see:"
echo "  /home/runner/work/Autonomous-Drone/Autonomous-Drone/ros2_ws/src/rtabmap_slam/README.md"
