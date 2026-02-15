#!/bin/bash

# Autonomous Drone Setup Script
# This script sets up the development environment for the Autonomous Drone project
# Compatible with Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic

set -e  # Exit on any error

# Use sudo only when not already root
SUDO="sudo"
if [ "$(id -u)" -eq 0 ]; then
    SUDO=""
fi

echo "=========================================="
echo "Autonomous Drone - Environment Setup"
echo "=========================================="

# Check if running on Ubuntu 24.04
if ! grep -q "Ubuntu 24.04" /etc/os-release; then
    echo "ERROR: This script is designed for Ubuntu 24.04"
    echo "Current OS: $(lsb_release -d | cut -f2)"
    exit 1
fi

echo "Ubuntu 24.04 detected"

# Update package list
echo "Updating package list..."
$SUDO apt update

# Install basic dependencies
echo "Installing basic dependencies..."
$SUDO apt install -y \
    curl \
    wget \
    gnupg \
    lsb-release \
    software-properties-common \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    git \
    build-essential \
    cmake \
    libeigen3-dev \
    libopencv-dev \
    python3-opencv \
    libg2o-dev \
    libgl1-mesa-dev \
    libgtk2.0-dev \
    pkg-config

echo "Basic dependencies installed"

locale | grep -q "UTF-8" || {
    echo "Setting locale to UTF-8..."
    $SUDO apt install -y locales
    $SUDO locale-gen en_US en_US.UTF-8
    $SUDO update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    echo "Locale set to UTF-8"
}

# Add Universe repository if not already added
echo "Ensuring Universe repository is enabled..."
$SUDO apt install software-properties-common
$SUDO add-apt-repository universe

# Install ros-dev-tools package
echo "Installing ros-dev-tools package..."
$SUDO apt update && $SUDO apt install -y ros-dev-tools

# Update APT repository caches
$SUDO apt update

# Install ROS 2 Jazzy
echo "Installing ROS 2 Jazzy..."
$SUDO apt update
$SUDO apt upgrade -y
$SUDO apt install -y ros-jazzy-desktop

# Source ROS 2 in bashrc if not already
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

echo "ROS 2 Jazzy installed"

# Install RTAB-Map ROS packages (required for SLAM launch)
echo "Installing RTAB-Map ROS 2 packages..."
$SUDO apt install -y \
    ros-jazzy-rtabmap \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-rtabmap-odom \
    ros-jazzy-rtabmap-slam \
    ros-jazzy-rtabmap-util \
    ros-jazzy-rtabmap-viz \
    ros-jazzy-rtabmap-msgs \
    ros-jazzy-rtabmap-conversions \
    ros-jazzy-rtabmap-python

$SUDO apt install -y \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    libpcl-dev

echo "RTAB-Map ROS packages installed"

# OMPL for RRT* 3D path planning (autonomy stack)
echo "Installing OMPL for RRT* planner..."
$SUDO apt install -y libompl-dev

echo "OMPL installed"

# Install ROS 2 Gazebo bridges and tools
echo "Installing ROS 2 Gazebo bridge and tools..."
$SUDO apt install -y \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-image \
    ros-jazzy-actuator-msgs \
    ros-jazzy-gps-msgs \
    ros-jazzy-vision-msgs

echo "ROS 2 Gazebo bridge installed"

# Install Gazebo Harmonic
echo "Installing Gazebo Harmonic..."
$SUDO wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -cs) main" | $SUDO tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
$SUDO apt update

# Handle DART library conflict between Ubuntu and OSRF packages
# Ubuntu provides libdart-collision-* packages that conflict with OSRF's libdart-core+collisions+odelcpsolver
# This is a known issue on Ubuntu 24.04 with the OSRF Gazebo repository
echo "Resolving DART library conflicts..."
$SUDO dpkg --remove --force-all \
  libdart6.13:amd64 \
  libdart-external-odelcpsolver6.13:amd64 \
  libdart-collision-bullet6.13:amd64 \
  libdart-collision-ode6.13:amd64 \
  2>/dev/null || true

# Install Gazebo with DART support
$SUDO apt install -y gz-harmonic

# Force install OSRF DART package if needed
if ! dpkg -l | grep -q "libdart-core+collisions+odelcpsolver6.13"; then
    echo "Installing OSRF DART libraries..."
    $SUDO apt install -y libdart-core+collisions+odelcpsolver6.13 --no-install-recommends
fi

# Configure any remaining packages
$SUDO dpkg --configure -a 2>/dev/null || true

# Clean up old DART dev packages that depend on removed libraries
$SUDO apt remove -y libdart-dev libdart-collision-bullet-dev libdart-collision-ode-dev \
  libdart-external-odelcpsolver-dev libdart-external-ikfast-dev \
  libdart-external-convhull-3d-dev libdart-utils-dev libdart-utils-urdf-dev \
  2>/dev/null || true

# Remove old Ignition packages that may have stray dependencies
$SUDO dpkg --remove --force-all libignition-physics5-dartsim-dev libignition-physics5-dev 2>/dev/null || true

# Final cleanup
$SUDO apt autoremove -y 2>/dev/null || true

echo "Gazebo Harmonic installed with DART dependencies resolved"

# Configure Gazebo command-line tools
# Note: GZ_CONFIG_PATH must be set to /usr/share/gz for the gz sim command to work properly
echo "Configuring Gazebo tools..."
echo "IMPORTANT: GZ_CONFIG_PATH environment variable is set in the launch files."
echo "If you run 'gz sim' directly, set: export GZ_CONFIG_PATH=/usr/share/gz"

# Create Python virtual environment in project directory (portable)
echo "Setting up Python virtual environment in project directory..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
VENV_DIR="$PROJECT_ROOT/.venv"

if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv "$VENV_DIR"
    echo "Virtual environment created at $VENV_DIR"
else
    echo "Virtual environment already exists at $VENV_DIR"
fi

# Activate venv and install Python packages
echo "Installing Python packages in virtual environment..."
source "$VENV_DIR/bin/activate"
pip install --upgrade pip setuptools wheel
# Install project-specific packages if needed
# pip install depthai==3.3.0
pip install argcomplete
# For colcon build (rosidl, etc.) when using venv Python
pip install numpy lark
deactivate

echo "Python packages installed in virtual environment"
echo "Virtual environment location: $VENV_DIR"
echo "This venv is portable and will work when the repo is cloned to any machine"

# Install Python argcomplete for ROS 2 shell completion (system-wide)
echo "Installing Python argcomplete (system-wide)..."
$SUDO apt install -y python3-argcomplete || true

echo "Python argcomplete installed"

echo ""
echo "=========================================="
echo "Verifying installations..."
echo "=========================================="

if command -v gz &> /dev/null; then
    GAZEBO_VERSION=$(gz sim --version 2>&1 | grep "version" | awk '{print $NF}')
    echo "✓ Gazebo Harmonic: $GAZEBO_VERSION"
else
    echo "⚠ Gazebo Harmonic: Not found in PATH"
fi

if [ -d "/opt/ros/jazzy" ]; then
    echo "✓ ROS 2 Jazzy: Installed"
else
    echo "⚠ ROS 2 Jazzy: Not found"
fi

echo ""
echo "=========================================="
echo "Environment setup complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Restart your terminal or run: source ~/.bashrc"
echo "2. Run: ./scripts/setup_workspace.sh"
echo ""
echo "For detailed instructions, see README.md"