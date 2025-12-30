#!/bin/bash
# Complete Project Setup Script for Autonomous Drone VIO System
# This script sets up the entire development environment

set -e

echo "=== Autonomous Drone VIO System Setup ==="
echo "This script will set up the complete development environment"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# Check if running on Ubuntu
if ! grep -q "Ubuntu" /etc/os-release; then
    print_error "This script is designed for Ubuntu. Please run on Ubuntu 22.04+"
    exit 1
fi

# Check ROS2 installation
print_header "Checking ROS2 Installation"
if ! command -v ros2 &> /dev/null; then
    print_error "ROS2 is not installed. Please install ROS2 Jazzy first:"
    echo "  sudo apt update && sudo apt install ros-jazzy-desktop"
    exit 1
fi

ROS_DISTRO=$(ros2 --version | grep -o 'jazzy\|humble\|iron')
if [ "$ROS_DISTRO" != "jazzy" ]; then
    print_warning "ROS2 $ROS_DISTRO detected. This project is optimized for Jazzy."
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi
print_status "ROS2 $ROS_DISTRO detected ✓"

# Install system dependencies
print_header "Installing System Dependencies"
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    libgtk2.0-dev \
    pkg-config \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libtbb2 \
    libtbb-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libdc1394-22-dev \
    libeigen3-dev \
    libboost-all-dev \
    libsuitesparse-dev \
    libopencv-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libgtest-dev \
    libfmt-dev \
    libpython3-dev \
    python3-dev \
    python3-numpy \
    python3-opencv

# Install ROS2-specific dependencies
print_header "Installing ROS2 Dependencies"
sudo apt install -y \
    ros-jazzy-pangolin \
    ros-jazzy-cv-bridge \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-rclcpp \
    ros-jazzy-image-transport \
    ros-jazzy-vision-msgs \
    ros-jazzy-depthai-ros

# Install Python dependencies
print_header "Installing Python Dependencies"
pip3 install \
    pymavlink \
    numpy \
    opencv-python \
    matplotlib \
    scipy

# Setup ORB-SLAM3
print_header "Setting up ORB-SLAM3"
if [ ! -d "$HOME/ORB_SLAM3" ] || [ ! -f "$HOME/ORB_SLAM3/lib/libORB_SLAM3.so" ]; then
    print_status "Installing ORB-SLAM3..."
    chmod +x setup_orbslam.sh
    ./setup_orbslam.sh
else
    print_status "ORB-SLAM3 already installed ✓"
fi

# Build ROS2 workspace
print_header "Building ROS2 Workspace"
cd ros2_ws

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Clean previous build if it exists
if [ -d "build" ]; then
    print_warning "Cleaning previous build..."
    rm -rf build install log
fi

# Build the workspace
print_status "Building ROS2 packages..."
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Test the build
print_header "Testing Build"
if ros2 pkg list | grep -q "orbslam3_bridge"; then
    print_status "orbslam3_bridge package built ✓"
else
    print_error "orbslam3_bridge package not found"
    exit 1
fi

if ros2 pkg list | grep -q "vio_bringup"; then
    print_status "vio_bringup package built ✓"
else
    print_error "vio_bringup package not found"
    exit 1
fi

if ros2 pkg list | grep -q "vio_mavlink_bridge"; then
    print_status "vio_mavlink_bridge package built ✓"
else
    print_error "vio_mavlink_bridge package not found"
    exit 1
fi

cd ..

# Create convenience scripts
print_header "Creating Convenience Scripts"
chmod +x *.sh
chmod +x ros2_ws/src/vio_bringup/scripts/*.py

print_status ""
print_status "🎉 Project setup completed successfully!"
print_status ""
print_status "Available commands:"
echo "  ./run_vio_simulation.sh    - Start VIO simulation with PX4 SITL"
echo "  ./start_vio_system.sh      - Start complete system (Gazebo + PX4 + ROS2)"
echo "  ./check_mavlink_vio.py     - Monitor MAVLink VIO messages"
echo "  ./check_slam_status.py     - Check ORB-SLAM3 status"
echo ""
print_status "Quick test:"
echo "  cd ros2_ws"
echo "  source install/setup.bash"
echo "  ros2 launch vio_bringup camera_simulation_no_hardware.launch.py"
echo ""
print_warning "Next steps:"
echo "1. Ensure PX4-Autopilot is cloned: git clone https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot"
echo "2. For hardware testing, ensure OAK-D camera is connected"
echo "3. Run ./run_vio_simulation.sh for the complete simulation pipeline"
echo ""
print_status "Setup complete! 🚁"

