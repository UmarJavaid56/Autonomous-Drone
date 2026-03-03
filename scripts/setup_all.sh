#!/bin/bash

# Quick Setup Script for Autonomous Drone
# Runs both dependency installation and workspace setup

set -e

echo "=========================================="
echo "Autonomous Drone - Quick Setup"
echo "=========================================="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Run dependency installation
echo "Step 1: Installing dependencies..."
"$SCRIPT_DIR/install_dependencies.sh"

# Setup Python virtual environment
echo "Step 2: Setting up Python virtual environment..."
"$SCRIPT_DIR/setup_venv.sh"

# Source ROS 2
echo "Step 3: Sourcing ROS 2..."
source /opt/ros/jazzy/setup.bash

# Run workspace setup
echo "Step 4: Setting up workspace..."
"$SCRIPT_DIR/setup_workspace.sh"

echo ""
echo "=========================================="
echo "Setup complete"
echo "=========================================="
echo ""
echo "To use the workspace:"
echo "  source $PROJECT_ROOT/ros2_ws/setup.sh"
echo ""
echo "Test the setup:"
echo "  ros2 launch x500_rtabmap_slam x500_rtabmap_slam.launch.py"