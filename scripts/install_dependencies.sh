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

# Install ROS 2 Jazzy
echo "Installing ROS 2 Jazzy..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop

# Source ROS 2 in bashrc if not already
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

echo "ROS 2 Jazzy installed"

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

# Install Python argcomplete for ROS 2 shell completion
echo "Installing Python argcomplete..."
sudo apt install -y python3-argcomplete || pip3 install argcomplete --user

# Remove stale argcomplete script if it exists
rm ~/.local/bin/register-python-argcomplete 2>/dev/null || true

echo "Python argcomplete installed"

# Install DepthAI
echo "Installing DepthAI..."
pip3 install depthai --user

echo "DepthAI installed"

# Install additional Python packages
pip3 install setuptools==58.2.0 --user

echo "Additional Python packages installed"

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