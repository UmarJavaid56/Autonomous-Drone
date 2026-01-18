#!/bin/bash

# Autonomous Drone Setup Script
# This script sets up the development environment for the Autonomous Drone project
# Compatible with Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic

set -e  # Exit on any error

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
sudo apt update

# Install basic dependencies
echo "Installing basic dependencies..."
sudo apt install -y \
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
    sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    echo "Locale set to UTF-8"
}

# Add Universe repository if not already added
echo "Ensuring Universe repository is enabled..."
sudo apt install software-properties-common
sudo add-apt-repository universe

# Install ros-dev-tools package
echo "Installing ros-dev-tools package..."
sudo apt update && sudo apt install -y ros-dev-tools

# Update APT repository caches
sudo apt update

# Install ROS 2 Jazzy
echo "Installing ROS 2 Jazzy..."
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop

# Source ROS 2 in bashrc if not already
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

echo "ROS 2 Jazzy installed"

# Install ROS 2 Gazebo bridges and tools
echo "Installing ROS 2 Gazebo bridge and tools..."
sudo apt install -y \
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
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update

# Handle DART library conflict between Ubuntu and OSRF packages
# Ubuntu provides libdart-collision-* packages that conflict with OSRF's libdart-core+collisions+odelcpsolver
# This is a known issue on Ubuntu 24.04 with the OSRF Gazebo repository
echo "Resolving DART library conflicts..."
sudo dpkg --remove --force-all \
  libdart6.13:amd64 \
  libdart-external-odelcpsolver6.13:amd64 \
  libdart-collision-bullet6.13:amd64 \
  libdart-collision-ode6.13:amd64 \
  2>/dev/null || true

# Install Gazebo with DART support
sudo apt install -y gz-harmonic

# Force install OSRF DART package if needed
if ! dpkg -l | grep -q "libdart-core+collisions+odelcpsolver6.13"; then
    echo "Installing OSRF DART libraries..."
    sudo apt install -y libdart-core+collisions+odelcpsolver6.13 --no-install-recommends
fi

# Configure any remaining packages
sudo dpkg --configure -a 2>/dev/null || true

# Clean up old DART dev packages that depend on removed libraries
sudo apt remove -y libdart-dev libdart-collision-bullet-dev libdart-collision-ode-dev \
  libdart-external-odelcpsolver-dev libdart-external-ikfast-dev \
  libdart-external-convhull-3d-dev libdart-utils-dev libdart-utils-urdf-dev \
  2>/dev/null || true

# Remove old Ignition packages that may have stray dependencies
sudo dpkg --remove --force-all libignition-physics5-dartsim-dev libignition-physics5-dev 2>/dev/null || true

# Final cleanup
sudo apt autoremove -y 2>/dev/null || true

echo "Gazebo Harmonic installed with DART dependencies resolved"

# Configure Gazebo command-line tools
# Note: GZ_CONFIG_PATH must be set to /usr/share/gz for the gz sim command to work properly
echo "Configuring Gazebo tools..."
echo "IMPORTANT: GZ_CONFIG_PATH environment variable is set in the launch files."
echo "If you run 'gz sim' directly, set: export GZ_CONFIG_PATH=/usr/share/gz"

# Create Python virtual environment for project dependencies
echo "Creating Python virtual environment..."
VENV_DIR="${HOME}/.venv-autonomous-drone"
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
pip install depthai==3.3.0
pip install argcomplete
deactivate

echo "Python packages installed in virtual environment"
echo "To use the environment, run: source ~/.venv-autonomous-drone/bin/activate"

# Install Python argcomplete for ROS 2 shell completion (system-wide)
echo "Installing Python argcomplete (system-wide)..."
sudo apt install -y python3-argcomplete || true

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