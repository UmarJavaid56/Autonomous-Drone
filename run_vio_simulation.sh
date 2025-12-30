#!/bin/bash
# VIO Simulation Setup Script
# This script sets up the complete VIO pipeline with proper ROS domain isolation

set -e

echo "=== VIO Simulation Setup ==="
echo "This script runs the complete VIO pipeline:"
echo "1. PX4 SITL (Terminal 1)"
echo "2. ROS2 VIO Pipeline (Terminal 2)"
echo ""
echo "Make sure you have:"
echo "- PX4-Autopilot cloned and built"
echo "- ROS2 Jazzy installed"
echo "- This workspace built: colcon build"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
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

# Check if PX4 is available
if [ ! -d "$HOME/PX4-Autopilot" ]; then
    print_error "PX4-Autopilot not found at $HOME/PX4-Autopilot"
    print_error "Please clone and build PX4-Autopilot first"
    exit 1
fi

# Check if ROS2 workspace is built
if [ ! -d "$HOME/Capstone/Autonomous-Drone/ros2_ws/install" ]; then
    print_error "ROS2 workspace not built. Run: cd ~/Capstone/Autonomous-Drone/ros2_ws && colcon build"
    exit 1
fi

echo ""
echo "=== Step 1: Start PX4 SITL (Terminal 1) ==="
echo "Run this in Terminal 1:"
echo ""
echo "cd ~/PX4-Autopilot"
echo "export ROS_DOMAIN_ID=0"
echo "make px4_sitl gz_x500"
echo ""
echo "Wait for PX4 to fully start up, then proceed to Step 2."
echo ""

echo "=== Step 2: Start ROS2 VIO Pipeline (Terminal 2) ==="
echo "Run this in Terminal 2:"
echo ""
echo "cd ~/Capstone/Autonomous-Drone/ros2_ws"
echo "source /opt/ros/jazzy/setup.bash"
echo "source install/setup.bash"
echo "export ROS_DOMAIN_ID=1"
echo "export ROS_LOCALHOST_ONLY=1"
echo "ros2 launch vio_bringup camera_simulation_no_hardware.launch.py"
echo ""

print_warning "IMPORTANT NOTES:"
echo "- Use ROS_DOMAIN_ID=0 for PX4 terminal"
echo "- Use ROS_DOMAIN_ID=1 for ROS2 terminal with ROS_LOCALHOST_ONLY=1"
echo "- This prevents MAVROS service conflicts"
echo "- The VIO bridge sends MAVLink directly to PX4 (port 14580)"
echo ""

read -p "Press Enter when ready to continue with the setup..."
