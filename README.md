# Autonomous-Drone

A ROS 2-based autonomous drone system with OAK-D camera integration, ORB-SLAM3 visual odometry, RTAB-Map SLAM, and Gazebo simulation.

## 🚀 Quick Start

### Prerequisites
- Ubuntu 24.04 LTS
- At least 8GB RAM, 4GB free disk space
- Internet connection for downloads

### One-Command Setup
```bash
git clone https://github.com/UmarJavaid56/Autonomous-Drone.git
cd Autonomous-Drone
./scripts/setup_all.sh
```

This will:
- Install ROS 2 Jazzy
- Install Gazebo Harmonic
- Install all dependencies
- Build the workspace

### Manual Setup (Alternative)

If you prefer step-by-step control:

```bash
# 1. Install system dependencies
./scripts/install_dependencies.sh

# 2. Source ROS 2
source /opt/ros/jazzy/setup.bash

# 3. Setup and build workspace
./scripts/setup_workspace.sh
```

## 🏗️ Architecture

- **ROS 2 Jazzy**: Latest LTS robotics framework
- **Gazebo Harmonic**: Modern simulation with Ogre2 rendering
- **OAK-D Camera**: DepthAI-powered stereo vision
- **ORB-SLAM3**: Real-time visual SLAM
- **RTAB-Map**: RGB-D SLAM for 3D mapping with point clouds
- **PX4 Integration**: MAVLink-based flight control

## 📁 Project Structure

```
Autonomous-Drone/
├── ros2_ws/                    # ROS 2 workspace
│   ├── src/
│   │   ├── depthai_cam/        # OAK-D camera ROS 2 driver
│   │   ├── orbslam3_bridge/    # ORB-SLAM3 ROS 2 bridge
│   │   ├── rtabmap_slam/       # RTAB-Map SLAM integration
│   │   ├── vio_perception/     # VIO perception pipeline
│   │   ├── vio_mavlink_bridge/ # MAVLink VIO bridge
│   │   └── vio_bringup/        # Launch configurations
│   └── external/               # External dependencies (ORB-SLAM3)
├── scripts/                    # Setup and utility scripts
├── config/                     # Configuration files
├── doc/                        # Documentation
└── SIMULATION_SETUP.md         # Simulation guide
```

## 🎯 Usage

### Launch Simulation
```bash
# Terminal 1: Launch Gazebo simulation with OAK-D camera
# Includes ROS 2 - Gazebo bridges for camera topics
ros2 launch depthai_cam depthai_cam_sim.launch.py

# Terminal 2: Launch ORB-SLAM3 with camera feed
ros2 launch vio_bringup vio_oakd_orbslam3.launch.py

# Terminal 3: Launch MAVROS SITL integration
ros2 launch vio_bringup vio_sitl_mavros.launch.py
```

**Note**: The simulation uses `ros_gz_bridge` to bridge Gazebo camera topics to ROS 2 topics. Topics published:
- `/oak_d_lite/rgb/image_raw` - RGB camera
- `/oak_d_lite/depth/image_raw` - Depth image
- `/oak_d_lite/depth/points` - Depth point cloud
- `/tf` - Transform frames

### RTAB-Map SLAM (3D Mapping)
```bash
# Launch x500_depth simulation with RTAB-Map SLAM
# This includes Gazebo simulation, RTAB-Map, and RViz visualization
ros2 launch rtabmap_slam x500_rtabmap_slam.launch.py

# For localization mode (using existing map)
ros2 launch rtabmap_slam x500_rtabmap_slam.launch.py localization:=true
```

RTAB-Map creates 3D point cloud maps and provides:
- Real-time 3D mapping using depth camera point clouds
- Visual odometry and pose estimation
- Loop closure detection for accurate mapping
- 2D occupancy grid for navigation planning

**Note**: First install RTAB-Map dependencies:
```bash
./scripts/install_rtabmap.sh
```

See [rtabmap_slam/README.md](ros2_ws/src/rtabmap_slam/README.md) for detailed usage and [RTAB-Map Quick Start](doc/RTABMAP_QUICKSTART.md) for step-by-step guide.

### Real Hardware
```bash
# Connect OAK-D camera and launch real-time VIO
ros2 launch vio_bringup vio_oakd_orbslam3.launch.py
```

## 🔧 Development

### Building
```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Testing
```bash
# Run all tests
colcon test
colcon test-result --verbose
```

## ⚠️ Troubleshooting

### Missing ros_gz_bridge Package

**Problem**: Launch fails with "package 'ros_gz_bridge' not found"

**Cause**: `ros_gz_bridge` is required to bridge Gazebo camera topics to ROS 2 but wasn't installed

**Solution**: Install the bridge package and dependencies
```bash
sudo apt install -y \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-image \
    ros-jazzy-actuator-msgs \
    ros-jazzy-gps-msgs \
    ros-jazzy-vision-msgs
```

Or run the updated installation script:
```bash
./scripts/install_dependencies.sh
```

### DART Library Conflict during Gazebo Installation

**Problem**: Installation fails with "trying to overwrite libdart-*.so.6.13.2" error

**Cause**: Ubuntu 24.04 provides conflicting DART physics library packages that conflict with OSRF's versions.

**Solution**: The updated installation scripts (`install_dependencies.sh`) automatically resolve this conflict by:
1. Removing conflicting Ubuntu DART packages
2. Installing OSRF's DART libraries
3. Cleaning up stray dependencies

If you encounter this issue after running the old scripts:
```bash
# Force remove conflicting packages
sudo dpkg --remove --force-all \
  libdart6.13:amd64 libdart-external-odelcpsolver6.13:amd64 \
  libdart-collision-bullet6.13:amd64 libdart-collision-ode6.13:amd64

# Install OSRF DART
sudo apt install -y libdart-core+collisions+odelcpsolver6.13

# Clean up
sudo dpkg --configure -a
sudo apt autoremove -y
```

### Gazebo `gz sim` Command Not Recognized

**Problem**: Running `gz sim` fails with "no such command" error

**Cause**: The `gz` command-line tool requires the `GZ_CONFIG_PATH` environment variable to locate plugin configurations.

**Solution**: Set the environment variable:
```bash
export GZ_CONFIG_PATH=/usr/share/gz
gz sim --version  # should now work
```

**Note**: This is automatically configured in ROS 2 launch files via `depthai_cam_sim.launch.py`, so you don't need to set it manually when launching through ROS 2.

### ROS 2 Shell Completion Warnings

**Problem**: "ModuleNotFoundError: No module named 'argcomplete.scripts'" on terminal startup

**Solution**: Remove the stale script:
```bash
rm ~/.local/bin/register-python-argcomplete
```

## 📚 Documentation

- [Simulation Setup Guide](SIMULATION_SETUP.md)
- [Camera Configuration](config/oakd_mono.yaml)
- [RTAB-Map Quick Start](doc/RTABMAP_QUICKSTART.md)
- [RTAB-Map Integration Guide](doc/RTABMAP_INTEGRATION.md)
- [API Documentation](doc/)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Run tests
5. Submit a pull request

## 📄 License

This project is licensed under the BSD-3-Clause License - see the package.xml files for details.

## 🙏 Acknowledgments

- [ROS 2](https://docs.ros.org/en/jazzy/)
- [Gazebo Sim](https://gazebosim.org/docs/harmonic/)
- [DepthAI](https://docs.luxonis.com/)
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)