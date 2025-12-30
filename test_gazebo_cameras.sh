#!/bin/bash

# Test script for Gazebo camera + depth + IMU setup

echo "Testing Gazebo camera setup..."

# Set environment
export PX4_GZ_STANDALONE=1
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=1

# Start Gazebo with the new camera model
echo "Starting Gazebo with x500_camera_depth model..."
gz sim -r -s ~/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf &
GZ_PID=$!
sleep 3

# Start PX4 SITL with the new model
echo "Starting PX4 SITL with x500_camera_depth..."
cd ~/PX4-Autopilot
make px4_sitl gz_x500_depth &
PX4_PID=$!
sleep 5

# Start ROS2 monitor
echo "Starting ROS2 camera monitor..."
cd ~/Capstone/Autonomous-Drone/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Waiting for camera topics..."
timeout 10 ros2 topic list | grep -E "camera|imu" || echo "No camera topics found"

echo ""
echo "Camera test complete."
echo "GZ_PID: $GZ_PID"
echo "PX4_PID: $PX4_PID"
echo ""
echo "To stop: kill $GZ_PID $PX4_PID"

# Keep running for manual testing
echo "Press Ctrl+C to exit..."
trap "echo 'Stopping...'; kill $GZ_PID $PX4_PID 2>/dev/null" INT
wait
