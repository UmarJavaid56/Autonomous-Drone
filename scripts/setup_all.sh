#!/bin/bash

# Quick Setup Script for Autonomous Drone
# Runs both dependency installation and workspace setup

set -e

echo "=========================================="
echo "Autonomous Drone - Quick Setup"
echo "=========================================="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Run dependency installation
echo "Step 1: Installing dependencies..."
"$SCRIPT_DIR/install_dependencies.sh"

# Source ROS 2
echo "Step 2: Sourcing ROS 2..."
source /opt/ros/jazzy/setup.bash

# Run workspace setup
echo "Step 3: Setting up workspace..."
"$SCRIPT_DIR/setup_workspace.sh"

echo ""
echo "=========================================="
echo "Setup complete"
echo "=========================================="
echo ""
echo "Test the setup:"
echo "ros2 launch depthai_cam depthai_cam_sim.launch.py"