#!/bin/bash

echo "=== Starting Complete VIO Autonomous Drone System ==="
echo ""

# Step 1: Let PX4 start Gazebo automatically (simplest approach)
echo "Step 1: Starting PX4 SITL (will auto-start Gazebo)..."
cd /home/umarj/PX4-Autopilot
PX4_GZ_WORLD=default make px4_sitl gz_x500_depth &
PX4_PID=$!
echo "PX4 + Gazebo starting (PID: $PX4_PID)"

# Wait for PX4 and Gazebo to initialize
echo "Waiting for PX4 and Gazebo to be ready..."
sleep 20

# Step 3: Start ROS2 VIO System
echo "Step 3: Starting ROS2 Camera SLAM system..."
cd /home/umarj/Capstone/Autonomous-Drone

# Setup DDS transport (fixes RViz image display issues)
source setup_dds_transport.sh

cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=1
ros2 launch vio_bringup gazebo_camera_vio.launch.py &
ROS_PID=$!
echo "ROS2 VIO system started (PID: $ROS_PID)"

# Step 4: Start QGroundControl
echo "Step 4: Starting QGroundControl..."
/home/umarj/QGroundControl-x86_64.AppImage &
QGC_PID=$!
echo "QGroundControl started (PID: $QGC_PID)"

echo ""
echo "=== SYSTEM STARTUP COMPLETE ==="
echo "• Gazebo: Physics simulation running"
echo "• PX4: Connected to Gazebo world"
echo "• ROS2: Camera SLAM processing active"
echo "• QGC: Flight control interface ready"
echo ""
echo "Your autonomous VIO drone is ready for flight!"
echo "Use QGroundControl to take off, fly, and test autonomous navigation."
echo ""
echo "To stop: kill $PX4_PID $ROS_PID $QGC_PID"

# Keep script running to show PIDs
echo "Press Ctrl+C to exit (system will continue running)"
trap "echo 'System still running in background. PIDs: PX4=$PX4_PID, ROS=$ROS_PID, QGC=$QGC_PID'" INT
wait
