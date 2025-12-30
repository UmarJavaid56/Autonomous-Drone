# VIO Bringup Package

This package provides launch configurations for the autonomous drone VIO (Visual-Inertial Odometry) system.

## MAVROS Status & Migration Path

**Current Status**: MAVROS has ROS2 parameter service conflicts that prevent reliable operation. The direct MAVLink bridge provides superior SITL integration.

**Future Migration**: When MAVROS ROS2 branches resolve parameter conflicts (check [MAVROS ROS2 Issues](https://github.com/mavlink/mavros/issues)), migrate by:

1. Update MAVROS: `git clone -b ros2 https://github.com/mavlink/mavros.git`
2. Replace `vio_mavlink_bridge_direct_node` with `vio_mavlink_bridge_node` in launch files
3. Add MAVROS node with minimal plugins: `plugin_whitelist: [vision_pose, sys_status]`

**Why Direct MAVLink is Better Now**:
- ✅ No parameter service conflicts
- ✅ Direct MAVLink communication to PX4
- ✅ Reliable SITL testing
- ✅ Same VISION_POSITION_ESTIMATE messages

## Architecture Overview

The system follows a **proper external process architecture**:

```
OAK-D Camera → depthai_cam (ROS2)
    ↓ [Images]
External ORB-SLAM3 Process (~/ORB_SLAM3/)
    ↓ [ENU Pose Estimates]
orb_slam3_bridge → /orbslam3/pose_processed (ENU)
    ↓ [ENU → NED conversion + Direct MAVLink]
vio_mavlink_bridge_direct → VISION_POSITION_ESTIMATE → PX4 SITL
    ↓ [Vision Position Fusion]
PX4 EKF2 → Improved State Estimation
```

**Note**: Using direct MAVLink bridge instead of MAVROS due to ROS2 parameter service conflicts in current MAVROS versions.

## Quick Start (Recommended)

For the fastest setup, run the automated setup script from the project root:

```bash
./setup_project.sh
```

This will install all dependencies, build ORB-SLAM3, and set up the ROS2 workspace automatically.

## Manual Setup (Advanced Users)

### ORB-SLAM3 Installation

**Option 1: Automated Setup (Recommended)**
```bash
./setup_orb_slam3.sh
```

**Option 2: Manual Setup**
```bash
# Install dependencies
sudo apt install -y ros-jazzy-pangolin build-essential cmake libeigen3-dev libopencv-dev

# Clone and build ORB-SLAM3
cd ~
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB-SLAM3

# Apply C++17 patches for modern compilers
sed -i 's/#include <mutex>/#include <mutex>\n#include <shared_mutex>/g' include/System.h
sed -i 's/std::mutex/std::shared_mutex/g' include/System.h
sed -i 's/std::unique_lock<std::mutex>/std::unique_lock<std::shared_mutex>/g' include/System.h

# Build
chmod +x build.sh
./build.sh

# Download vocabulary
cd Vocabulary
wget https://github.com/UZ-SLAMLab/ORB_SLAM3/raw/master/Vocabulary/ORBvoc.txt.tar.gz
tar -xf ORBvoc.txt.tar.gz
```

### Why ORB-SLAM3 is External

**Current Approach**: ORB-SLAM3 installs at `~/ORB_SLAM3` (external to this repository)
- ✅ **Easier sharing**: Others can clone and run `./setup_project.sh`
- ✅ **No repository bloat**: Large binaries not tracked in git
- ✅ **Independent updates**: Upgrade ORB-SLAM3 without affecting project
- ✅ **Reusability**: Same ORB-SLAM3 can serve multiple projects
- ✅ **Industry standard**: Follows ROS2 best practices for external dependencies

**Alternative Approach**: Workspace-relative installation (like commit 084d6d4c)
- ❌ **Sharing friction**: Others must manually install ORB-SLAM3
- ❌ **Repository bloat**: Large binaries in git history
- ❌ **Version coupling**: ORB-SLAM3 updates affect project commits

## Launch Files

### `camera_simulation.launch.py`
- **Purpose**: Testing with mock SLAM data
- **Use**: Development and simulation testing
- **Components**: Camera + Mock SLAM + VIO Bridge + MAVROS

### `camera_simulation_no_hardware.launch.py`
- **Purpose**: Simulation without camera hardware
- **Use**: Software development and testing
- **Components**: Mock SLAM + VIO Bridge + MAVROS

### `external_orbslam3.launch.py`
- **Purpose**: Production setup with real ORB-SLAM3
- **Use**: When you have camera + ORB-SLAM3 working
- **Components**: Camera + ORB-SLAM3 Bridge + VIO Bridge + MAVROS

## Usage Examples

### Simulation Testing (Current Working State)
```bash
# Terminal 1: PX4 SITL
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# Terminal 2: ROS2 Pipeline
cd ~/Capstone/Autonomous-Drone/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch vio_bringup camera_simulation_no_hardware.launch.py
```

### With Real ORB-SLAM3 (Future State)
```bash
# Terminal 1: PX4 SITL
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# Terminal 2: External ORB-SLAM3
cd ~/ORB_SLAM3
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml /path/to/images/

# Terminal 3: ROS2 Bridge Pipeline
cd ~/Capstone/Autonomous-Drone/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch vio_bringup external_orbslam3.launch.py
```

## Coordinate Frames

- **ORB-SLAM3 Output**: ENU (East-North-Up)
- **PX4 Input**: NED (North-East-Down)
- **Conversion**: `vio_mavlink_bridge_node` handles ENU→NED transformation

## ROS2 Topics

- `/orbslam3/pose` - ORB-SLAM3 pose estimates (ENU)
- `/mavros/vision_pose/pose` - MAVROS vision pose (NED)
- `/mavros/state` - MAVROS system state
- `/mavros/local_position/pose` - PX4 local position

## Next Steps

1. **Get ORB-SLAM3 Building** - Apply C++17 patches and resolve dependencies
2. **Test Camera Integration** - Verify OAK-D → ORB-SLAM3 data flow
3. **Validate Coordinate Conversion** - Ensure proper ENU→NED transformation
4. **Performance Testing** - Measure latency and accuracy
5. **Hardware Integration** - Connect with real Pixhawk 6C

## Troubleshooting

### ORB-SLAM3 Build Issues
- Ensure C++17 support (patches applied)
- Install Pangolin: `sudo apt install ros-jazzy-pangolin`
- Check Eigen version compatibility

### ROS2 Communication Issues
- Verify topic names match between processes
- Check MAVROS connection to PX4
- Monitor `/rosout` for error messages

### Coordinate Frame Issues
- Confirm ORB-SLAM3 outputs ENU coordinates
- Verify NED conversion in `vio_mavlink_bridge_node`
- Test with known trajectories
