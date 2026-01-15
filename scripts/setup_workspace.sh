#!/bin/bash

# Workspace Setup Script for Autonomous Drone
# This script builds the ROS 2 workspace

set -e  # Exit on any error

echo "=========================================="
echo "Autonomous Drone - Workspace Setup"
echo "=========================================="

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS 2 is not sourced. Run: source /opt/ros/jazzy/setup.bash"
    exit 1
fi

if [ "$ROS_DISTRO" != "jazzy" ]; then
    echo "ERROR: ROS 2 Jazzy is required. Current: $ROS_DISTRO"
    exit 1
fi

echo "ROS 2 Jazzy detected"

# Navigate to workspace
cd "$(dirname "$0")/.."
WORKSPACE_DIR="$(pwd)"

if [ ! -d "ros2_ws/src" ]; then
    echo "ERROR: ros2_ws/src directory not found"
    exit 1
fi

echo "Setting up workspace in: $WORKSPACE_DIR"

# Clean previous builds
echo "Cleaning previous builds..."
rm -rf ros2_ws/build ros2_ws/install ros2_ws/log

# Install dependencies
echo "Installing ROS 2 dependencies..."
cd ros2_ws
rosdep init || true  # May already be initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
echo "Building workspace..."
colcon build --symlink-install

echo "✓ Workspace built successfully"

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

# Add to bashrc if not already
if ! grep -q "source $WORKSPACE_DIR/ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "source $WORKSPACE_DIR/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

echo ""
echo "=========================================="
echo "Workspace setup complete!"
echo "=========================================="
echo ""
echo "To use the workspace:"
echo "1. Restart terminal or run: source ~/.bashrc"
echo "2. Launch simulation: ros2 launch depthai_cam depthai_cam_sim.launch.py"
echo ""
echo "For more information, see README.md"