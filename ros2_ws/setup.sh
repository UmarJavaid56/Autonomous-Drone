#!/usr/bin/env bash
# Setup script for ROS 2 workspace with Python virtual environment
# Portable - works from any location when repo is cloned

# Don't use `set -u` here: ROS/colcon-generated setup files may reference
# unset variables (e.g. COLCON_TRACE).
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${WS_DIR}/.." && pwd)"
VENV_DIR="${PROJECT_ROOT}/.venv"

# Activate virtual environment if it exists
if [ -f "${VENV_DIR}/bin/activate" ]; then
  # shellcheck disable=SC1090
  source "${VENV_DIR}/bin/activate"
  echo "Activated Python virtual environment: ${VENV_DIR}"
else
  echo "WARNING: Virtual environment not found at ${VENV_DIR}"
  echo "Run: ./scripts/setup_venv.sh to create it"
fi

# Source workspace if it exists
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "${WS_DIR}/install/setup.bash"
  echo "Sourced ROS 2 workspace: ${WS_DIR}"
else
  echo "WARNING: Workspace not built. Run: ./scripts/setup_workspace.sh"
fi

