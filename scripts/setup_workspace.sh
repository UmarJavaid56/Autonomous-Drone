#!/bin/bash

# Workspace Setup Script for Autonomous Drone
# This script builds the ROS 2 workspace

set -e  # Exit on any error

echo "=========================================="
echo "Autonomous Drone - Workspace Setup"
echo "=========================================="

# Get project root directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="$PROJECT_ROOT/.venv"

# Setup Python virtual environment
echo "Setting up Python virtual environment..."
if [ ! -d "$VENV_DIR" ]; then
    "$SCRIPT_DIR/setup_venv.sh"
fi

# Activate virtual environment
if [ -f "$VENV_DIR/bin/activate" ]; then
    echo "Activating Python virtual environment..."
    source "$VENV_DIR/bin/activate"
else
    echo "WARNING: Virtual environment not found. Creating it..."
    "$SCRIPT_DIR/setup_venv.sh"
    source "$VENV_DIR/bin/activate"
fi

# Verify venv is active
if [ -z "$VIRTUAL_ENV" ]; then
    echo "ERROR: Failed to activate virtual environment"
    exit 1
fi
echo "Virtual environment active: $VIRTUAL_ENV"

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
cd "$PROJECT_ROOT"
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

# Build the workspace with venv Python
echo "Building workspace with virtual environment Python..."
# Use the venv Python explicitly to avoid system setuptools issues
PYTHONNOUSERSITE=1 \
PYTHONPATH="$VIRTUAL_ENV/lib/python$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')/site-packages:$PYTHONPATH" \
colcon build --symlink-install

echo "✓ Workspace built successfully"

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

# Ensure setup.sh exists and is up to date (already created, but verify)
SETUP_SCRIPT="$WORKSPACE_DIR/ros2_ws/setup.sh"
if [ ! -f "$SETUP_SCRIPT" ]; then
    cat > "$SETUP_SCRIPT" << 'EOF'
#!/usr/bin/env bash
# Setup script for ROS 2 workspace with Python virtual environment
# This script activates the venv and sources the workspace
# Portable - works from any location when repo is cloned

# Get the project root (parent of ros2_ws)
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$WS_DIR/.." && pwd)"
VENV_DIR="$PROJECT_ROOT/.venv"

# Activate virtual environment if it exists
if [ -f "$VENV_DIR/bin/activate" ]; then
    source "$VENV_DIR/bin/activate"
    echo "Activated Python virtual environment: $VENV_DIR"
else
    echo "WARNING: Virtual environment not found at $VENV_DIR"
    echo "Run: ./scripts/setup_venv.sh to create it"
fi

# Source workspace if it exists
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
    echo "Sourced ROS 2 workspace: $WS_DIR"
else
    echo "WARNING: Workspace not built. Run: ./scripts/setup_workspace.sh"
fi
EOF
    chmod +x "$SETUP_SCRIPT"
fi

echo ""
echo "=========================================="
echo "Workspace setup complete!"
echo "=========================================="
echo ""
echo "To use the workspace:"
echo "1. Source the workspace setup script:"
echo "   source $SETUP_SCRIPT"
echo ""
echo "2. Or manually activate venv and source workspace:"
echo "   source $VENV_DIR/bin/activate"
echo "   source $WORKSPACE_DIR/ros2_ws/install/setup.bash"
echo ""
echo "3. Launch simulation:"
echo "   ros2 launch x500_rtabmap_slam x500_rtabmap_slam.launch.py"
echo ""
echo "For more information, see README.md"