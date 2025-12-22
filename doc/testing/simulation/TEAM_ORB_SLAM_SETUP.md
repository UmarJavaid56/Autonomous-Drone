# ORB-SLAM3 Team Access Guide

## Overview

ORB-SLAM3 is set up as an **external dependency** for the autonomous drone VIO system. This guide explains how team members can access and use the library.

## Quick Start for Team Members

### Automated Setup (Recommended)
```bash
cd ~/Capstone/Autonomous-Drone
./setup_orbslam.sh
```

This script handles everything automatically:
- Installs dependencies
- Clones ORB-SLAM3
- Applies compatibility patches
- Builds the library
- Configures environment
- Verifies integration

### Manual Setup (Alternative)
If you prefer manual control:

```bash
# 1. Install dependencies
sudo apt install -y ros-jazzy-pangolin libeigen3-dev

# 2. Clone and build
cd ~
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
./build.sh  # May need patches from setup script

# 3. Configure environment
echo 'export ORB_SLAM3_PATH=$HOME/ORB_SLAM3' >> ~/.bashrc
source ~/.bashrc
```

## Architecture Understanding

### Why External ORB-SLAM3?
```
Wrong: ORB-SLAM3 embedded in ROS2 workspace
Correct: ORB-SLAM3 as external service

Benefits:
• Clean separation of concerns
• Independent updates
• Professional architecture
• Easy team collaboration
```

### System Architecture
```
OAK-D Camera → depthai_cam (ROS2)
    ↓ [Images]
External ORB-SLAM3 Process (~/ORB_SLAM3/)
    ↓ [ENU Pose Estimates]
orb_slam3_bridge → /orbslam3/pose (ROS2 topic)
    ↓ [ENU→NED conversion]
vio_mavlink_bridge → /mavros/vision_pose/pose
    ↓ [MAVLink messages]
PX4 SITL → Vision Position Fusion
```

## File Locations

After setup, your system will have:

```
~/
├── ORB_SLAM3/                    # External SLAM library
│   ├── lib/libORB_SLAM3.so       # Main library (5.2MB)
│   ├── include/                  # Headers
│   └── Vocabulary/               # ORB dictionary
│
└── Capstone/Autonomous-Drone/
    ├── setup_orbslam.sh          # Team setup script
    ├── verify_orbslam.sh         # Verification script
    └── ros2_ws/                  # ROS2 workspace
        └── src/vio_mavlink_bridge/ # ROS2 integration
```

## Testing the Setup

### Quick Verification
```bash
# Run verification script
~/verify_orbslam.sh
```

### Simulation Test (No Hardware Required)
```bash
# Terminal 1: PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# Terminal 2: ROS2 Pipeline
cd ~/Capstone/Autonomous-Drone/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch vio_bringup camera_simulation_no_hardware.launch.py
```

### Expected Results
- ORB-SLAM3 mock poses at 30Hz on `/orbslam3/pose`
- MAVROS vision poses at 30Hz on `/mavros/vision_pose/pose`
- PX4 console shows "Vision position fusion active"

## Troubleshooting

### Build Failures

#### Out of Memory (OOM)
```bash
# Check for OOM killer
dmesg | tail -5 | grep -i "killed\|oom"

# Solution: Build with single thread
cd ~/ORB_SLAM3/build
make clean && make -j1
```

#### Missing Dependencies
```bash
# Reinstall dependencies
sudo apt install -y ros-jazzy-pangolin libeigen3-dev libopencv-dev
```

#### CMake Errors
```bash
# Clean rebuild
cd ~/ORB_SLAM3
rm -rf build
./setup_orbslam.sh  # Re-run setup script
```

### Runtime Issues

#### Library Not Found
```bash
# Check environment
echo $ORB_SLAM3_PATH
ls -la $ORB_SLAM3_PATH/lib/

# Re-source environment
source ~/.bashrc
```

#### ROS2 Integration Problems
```bash
# Clean rebuild ROS2 workspace
cd ~/Capstone/Autonomous-Drone/ros2_ws
rm -rf build install log
colcon build --packages-select vio_mavlink_bridge
```

## Performance Specifications

- **Library Size**: 5.2MB (optimized)
- **Memory Usage**: ~50MB at runtime
- **CPU Usage**: Minimal when idle
- **Update Rate**: 30Hz pose estimation
- **Compatibility**: ROS2 Jazzy, Ubuntu 24.04

## Team Collaboration Features

### Automated Setup
- `setup_orbslam.sh` - One-command setup for new team members
- `verify_orbslam.sh` - Automated verification of installation
- Environment auto-configuration

### Version Management
- ORB-SLAM3 external (not in repo)
- Patches applied via setup script
- Consistent builds across team

### Documentation
- This guide for reference
- Inline comments in setup scripts
- Troubleshooting section

## Development Workflow

### For Simulation Development
1. Use mock SLAM: `camera_simulation_no_hardware.launch.py`
2. Develop algorithms without camera hardware
3. Test coordinate transformations
4. Validate PX4 integration

### For Hardware Integration
1. Setup real camera: `external_orbslam3.launch.py`
2. Replace mock SLAM with real ORB-SLAM3 process
3. Test with actual camera data
4. Validate autonomous flight

## Advanced Usage

### Custom ORB-SLAM3 Builds
```bash
# Modify settings in ~/ORB_SLAM3/
# Rebuild with: cd ~/ORB_SLAM3 && ./build.sh
# ROS2 will automatically use updated library
```

### Multiple Team Members
```bash
# Each team member runs setup script independently
# No conflicts - ORB-SLAM3 builds are local
# Share only the setup scripts and documentation
```

### CI/CD Integration
```bash
# For automated testing
# Use Docker with pre-built ORB-SLAM3
# Or integrate setup script into CI pipeline
```

## Support and Resources

### Getting Help
1. Run `~/verify_orbslam.sh` first
2. Check this documentation
3. Re-run setup script: `./setup_orbslam.sh`
4. Check system resources (8GB+ RAM recommended)

### Key Files to Know
- `setup_orbslam.sh` - Complete setup automation
- `verify_orbslam.sh` - Installation verification
- `vio_mavlink_bridge/CMakeLists.txt` - ROS2 integration
- `vio_bringup/launch/` - Launch configurations

### Related Documentation
- `vio_bringup/README.md` - Complete system overview
- PX4 Vision documentation
- ROS2 MAVROS documentation

---

**Status**: ORB-SLAM3 ready for team use
**Setup Method**: `./setup_orbslam.sh` (automated)
**Integration**: Automatic via ROS2 CMake
**Testing**: `ros2 launch vio_bringup camera_simulation_no_hardware.launch.py`

