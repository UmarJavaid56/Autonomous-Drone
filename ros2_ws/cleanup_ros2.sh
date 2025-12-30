#!/bin/bash
# ROS2 Cleanup Script
# Run this to clean up ROS2 state before starting fresh

echo "Cleaning up ROS2 processes and state..."

# Kill all ROS2 related processes
pkill -9 -f ros2
pkill -9 -f mavros
pkill -9 -f python3
pkill -9 -f gz
pkill -9 -f px4

# Wait for processes to die
sleep 3

# Clean up ROS2 state files
rm -rf ~/.ros/*
rm -rf /tmp/ros2*
rm -rf /tmp/launch_params_*
rm -rf /dev/shm/*ros2*
rm -rf /dev/shm/*dds*

echo "Cleanup complete. Ready to start fresh."
