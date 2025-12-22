# VIO Bringup Package

This package provides launch configurations for the autonomous drone VIO (Visual-Inertial Odometry) system.

## Architecture Overview

The system follows a **proper external process architecture**:

```
OAK-D Camera → depthai_cam (ROS2)
    ↓ [Images]
External ORB-SLAM3 Process (~/ORB_SLAM3/)
    ↓ [ENU Pose Estimates]
orb_slam3_bridge → /orbslam3/pose (ROS2 topic)
    ↓ [ENU → NED conversion]
vio_mavlink_bridge → /mavros/vision_pose/pose
    ↓ [MAVLink messages]
PX4 SITL → Vision Position Fusion
```

## Important: ORB-SLAM3 Runs Externally

**ORB-SLAM3 is NOT part of this ROS2 workspace.** It runs as a separate external process.

### Why External?
- ORB-SLAM3 is a standalone SLAM library
- Avoids repository bloat and licensing issues
- Allows independent updates and testing
- Follows industry best practices

### Setup Instructions

1. **Install Dependencies:**
   ```bash
   sudo apt install -y ros-jazzy-pangolin
   ```

2. **Clone ORB-SLAM3 Externally:**
   ```bash
   cd ~
   git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
   cd ORB_SLAM3
   # Apply C++17 patches (see main README)
   ./build.sh
   ```

3. **Download Vocabulary:**
   ```bash
   cd ~/ORB_SLAM3
   cd Vocabulary
   wget https://github.com/UZ-SLAMLab/ORB_SLAM3/raw/master/Vocabulary/ORBvoc.txt.tar.gz
   tar -xf ORBvoc.txt.tar.gz
   ```

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
