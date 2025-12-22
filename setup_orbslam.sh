#!/bin/bash

# ORB-SLAM3 Complete Setup Script for Autonomous Drone Team
# This script handles the complete setup of ORB-SLAM3 as an external dependency
# Run with: ./setup_orbslam.sh

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
ORB_SLAM_REPO="https://github.com/UZ-SLAMLab/ORB_SLAM3.git"
ORB_SLAM_DIR="$HOME/ORB_SLAM3"
REQUIRED_ROS_VERSION="jazzy"

echo -e "${BLUE}=== ORB-SLAM3 Autonomous Drone Setup ===${NC}"
echo -e "${BLUE}Setting up external ORB-SLAM3 library for VIO navigation${NC}"
echo ""

# Function to print status messages
print_status() {
    echo -e "${GREEN}[OK] $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}[WARNING] $1${NC}"
}

print_error() {
    echo -e "${RED}[ERROR] $1${NC}"
}

print_info() {
    echo -e "${BLUE}[INFO] $1${NC}"
}

# Check if running as root (don't allow)
if [[ $EUID -eq 0 ]]; then
    print_error "This script should not be run as root. Please run as a regular user."
    exit 1
fi

# Check for ROS2
print_info "Checking ROS2 installation..."
if ! command -v ros2 &> /dev/null; then
    print_error "ROS2 not found. Please install ROS2 $REQUIRED_ROS_VERSION first."
    print_info "Visit: https://docs.ros.org/en/$REQUIRED_ROS_VERSION/Installation.html"
    exit 1
fi

# Check ROS2 version
ROS_VERSION=$(ros2 --version | grep -o "$REQUIRED_ROS_VERSION" || echo "")
if [[ "$ROS_VERSION" != "$REQUIRED_ROS_VERSION" ]]; then
    print_warning "ROS2 version might not be $REQUIRED_ROS_VERSION. This script is designed for ROS2 $REQUIRED_ROS_VERSION."
fi

print_status "ROS2 detected"

# Check available memory
print_info "Checking system resources..."
TOTAL_MEM=$(free -m | awk 'NR==2{printf "%.0f", $2}')
if [[ $TOTAL_MEM -lt 8000 ]]; then
    print_warning "System has ${TOTAL_MEM}MB RAM. ORB-SLAM3 compilation requires 8GB+ RAM."
    print_warning "Consider using a system with more memory or building with -j1 flag."
fi

print_status "System resources OK (${TOTAL_MEM}MB RAM)"

# Install system dependencies
print_info "Installing system dependencies..."
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libopencv-dev \
    python3-dev \
    ros-$REQUIRED_ROS_VERSION-pangolin

print_status "System dependencies installed"

# Clone ORB-SLAM3 if not exists
print_info "Setting up ORB-SLAM3..."
if [[ -d "$ORB_SLAM_DIR" ]]; then
    print_warning "ORB_SLAM3 directory already exists at $ORB_SLAM_DIR"
    print_info "Updating existing repository..."
    cd "$ORB_SLAM_DIR"
    git pull origin master
else
    print_info "Cloning ORB-SLAM3 repository..."
    git clone "$ORB_SLAM_REPO" "$ORB_SLAM_DIR"
    cd "$ORB_SLAM_DIR"
fi

print_status "ORB-SLAM3 repository ready"

# Apply compatibility patches
print_info "Applying ROS2 $REQUIRED_ROS_VERSION compatibility patches..."

# Main CMakeLists.txt patches
print_info "Patching main CMakeLists.txt..."
sed -i 's/set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")/set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")/g' CMakeLists.txt
sed -i 's/-std=c++17/-std=c++14/g' CMakeLists.txt
sed -i 's/COMPILEDWITHC17/COMPILEDWITHC14/g' CMakeLists.txt
sed -i '/find_package(Eigen3 3.1.0 REQUIRED)/a # Pangolin\nset(Pangolin_DIR "/opt/ros/'$REQUIRED_ROS_VERSION'/lib/x86_64-linux-gnu/cmake/Pangolin")' CMakeLists.txt

# Third-party library patches
print_info "Patching third-party libraries..."

# g2o
sed -i 's/-std=c++17/-std=c++14/g' Thirdparty/g2o/CMakeLists.txt
sed -i 's/COMPILEDWITHC17/COMPILEDWITHC14/g' Thirdparty/g2o/CMakeLists.txt

# Sophus
sed -i 's/set(CMAKE_CXX_STANDARD 17)/set(CMAKE_CXX_STANDARD 14)/g' Thirdparty/Sophus/CMakeLists.txt

# DBoW2
sed -i 's/COMPILEDWITHC17/COMPILEDWITHC14/g' Thirdparty/DBoW2/CMakeLists.txt

print_status "Compatibility patches applied"

# Build ORB-SLAM3
print_info "Building ORB-SLAM3 library..."
print_warning "This may take 5-15 minutes depending on your system..."

# Clean any previous build
rm -rf build
mkdir build
cd build

# Configure with CMake
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build with memory-conscious settings
if [[ $TOTAL_MEM -lt 8000 ]]; then
    print_warning "Building with single thread due to limited RAM..."
    make -j1
else
    make -j$(nproc)
fi

# Verify build
cd ..
if [[ ! -f "lib/libORB_SLAM3.so" ]]; then
    print_error "Build failed - libORB_SLAM3.so not found"
    print_info "Check the build output above for errors"
    print_info "Try: cd $ORB_SLAM_DIR/build && make VERBOSE=1"
    exit 1
fi

LIB_SIZE=$(stat -c%s "lib/libORB_SLAM3.so" | numfmt --to=iec-i --suffix=B)
print_status "ORB-SLAM3 library built successfully (${LIB_SIZE})"

# Update shell environment
print_info "Configuring environment..."

# Add to .bashrc if not already present
BASHRC_ENTRY="export ORB_SLAM3_PATH=\$HOME/ORB_SLAM3"
if ! grep -q "ORB_SLAM3_PATH" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ORB-SLAM3 for autonomous drone project" >> ~/.bashrc
    echo "$BASHRC_ENTRY" >> ~/.bashrc
    print_status "Added ORB_SLAM3_PATH to ~/.bashrc"
else
    print_info "ORB_SLAM3_PATH already configured in ~/.bashrc"
fi

# Source the environment
export ORB_SLAM3_PATH="$HOME/ORB_SLAM3"
print_status "Environment configured"

# Test ROS2 integration
print_info "Testing ROS2 workspace integration..."

# Check if ROS2 workspace exists and is built
ROS2_WS="$HOME/Capstone/Autonomous-Drone/ros2_ws"
if [[ -d "$ROS2_WS" ]]; then
    cd "$ROS2_WS"

    # Source ROS2
    source "/opt/ros/$REQUIRED_ROS_VERSION/setup.bash"

    # Build if needed
    if [[ ! -d "install" ]]; then
        print_info "Building ROS2 workspace..."
        colcon build --packages-select vio_mavlink_bridge vio_bringup
    fi

    # Quick test
    if [[ -d "install" ]]; then
        source install/setup.bash
        if ros2 pkg list | grep -q "vio_mavlink_bridge"; then
            print_status "ROS2 integration verified"
        else
            print_warning "ROS2 package not found, but workspace exists"
        fi
    fi
else
    print_warning "ROS2 workspace not found at $ROS2_WS"
    print_info "Make sure you have cloned the autonomous-drone repository"
fi

# Create verification script
print_info "Creating verification script..."
cat > "$HOME/verify_orbslam.sh" << 'EOF'
#!/bin/bash
echo "=== ORB-SLAM3 Verification ==="

# Check library
if [[ -f "$HOME/ORB_SLAM3/lib/libORB_SLAM3.so" ]]; then
    echo "Library found: $(stat -c%s $HOME/ORB_SLAM3/lib/libORB_SLAM3.so | numfmt --to=iec-i --suffix=B)"
else
    echo "Library not found"
    exit 1
fi

# Check environment
if [[ -n "$ORB_SLAM3_PATH" ]]; then
    echo "Environment variable set: $ORB_SLAM3_PATH"
else
    echo "Environment variable not set"
fi

# Check ROS2 (if available)
if command -v ros2 &> /dev/null; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    if ros2 pkg list 2>/dev/null | grep -q "vio_mavlink_bridge"; then
        echo "ROS2 integration ready"
    else
        echo "ROS2 workspace needs building"
    fi
fi

echo ""
echo "=== Test Commands ==="
echo "# Quick simulation test:"
echo "ros2 launch vio_bringup camera_simulation_no_hardware.launch.py"
echo ""
echo "# Full system test:"
echo "cd ~/PX4-Autopilot && make px4_sitl gz_x500 &"
echo "ros2 launch vio_bringup camera_simulation_no_hardware.launch.py"
EOF

chmod +x "$HOME/verify_orbslam.sh"

print_status "Verification script created: ~/verify_orbslam.sh"

# Final summary
echo ""
echo -e "${GREEN}ORB-SLAM3 Setup Complete!${NC}"
echo ""
echo -e "${BLUE}What was installed:${NC}"
echo "  - ORB-SLAM3 library at: $ORB_SLAM_DIR/lib/libORB_SLAM3.so"
echo "  - System dependencies (Pangolin, Eigen, OpenCV)"
echo "  - Environment configuration"
echo "  - Compatibility patches for ROS2 $REQUIRED_ROS_VERSION"
echo ""
echo -e "${BLUE}Next steps:${NC}"
echo "  1. Run: source ~/.bashrc"
echo "  2. Verify: ~/verify_orbslam.sh"
echo "  3. Test: ros2 launch vio_bringup camera_simulation_no_hardware.launch.py"
echo ""
echo -e "${BLUE}For team members:${NC}"
echo "  - Share this script: ~/Capstone/Autonomous-Drone/setup_orbslam.sh"
echo "  - Documentation: ~/Capstone/Autonomous-Drone/docs/TEAM_ORB_SLAM_SETUP.md"
echo ""
echo -e "${YELLOW}Note:${NC} ORB-SLAM3 runs as external process - not part of ROS2 workspace"
echo -e "${YELLOW}Note:${NC} Mock SLAM is used for simulation until real camera hardware arrives"

# Final verification
"$HOME/verify_orbslam.sh"

echo ""
echo -e "${GREEN}Setup complete! Ready for autonomous drone development.${NC}"
