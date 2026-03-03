#!/usr/bin/env bash
#
# Launch x500 RTAB-Map SLAM (simulation + RViz)
#
# Usage:
#   ./scripts/launch_rtabmap_slam.sh [ros2 launch arguments...]
#
# Examples:
#   ./scripts/launch_rtabmap_slam.sh
#   ./scripts/launch_rtabmap_slam.sh rviz:=false
#   ./scripts/launch_rtabmap_slam.sh localization:=true
#
# Notes:
# - Uses a project-local ROS log directory to avoid ~/.ros permission issues.
# - Sets PYTHONNOUSERSITE=1 to avoid broken user-site setuptools on Python 3.12.
# - Automatically builds the workspace if needed before launching.
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${PROJECT_ROOT}/ros2_ws"

# Setup ROS environment variables
mkdir -p "${PROJECT_ROOT}/.ros/log"
export ROS_HOME="${PROJECT_ROOT}/.ros"
export ROS_LOG_DIR="${PROJECT_ROOT}/.ros/log"
export APPORT_DISABLE=1
export PYTHONNOUSERSITE=1

# Source ROS 2
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "ERROR: ROS 2 Jazzy not found at /opt/ros/jazzy/"
    echo "Please install ROS 2 Jazzy or update the path in this script."
    exit 1
fi
source /opt/ros/jazzy/setup.bash

# Check if workspace is built, build if needed
if [ ! -f "${WS_DIR}/install/setup.bash" ]; then
    echo "Workspace not built. Building workspace..."
    cd "${WS_DIR}"
    PYTHONNOUSERSITE=1 colcon build --symlink-install
    cd "${PROJECT_ROOT}"
fi

# Source workspace
source "${WS_DIR}/install/setup.bash"

# Build the RTAB-Map package (in case of recent changes)
echo "Building x500_rtabmap_slam package..."
cd "${WS_DIR}"
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select x500_rtabmap_slam
cd "${PROJECT_ROOT}"

# Re-source after build to pick up any changes
source "${WS_DIR}/install/setup.bash"

# Launch RTAB-Map SLAM
echo "Launching RTAB-Map SLAM..."
exec ros2 launch x500_rtabmap_slam x500_rtabmap_slam.launch.py "$@"


