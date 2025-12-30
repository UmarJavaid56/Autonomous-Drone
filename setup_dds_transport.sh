#!/bin/bash

# Fix DDS Transport Issues for ROS2
# This script configures Fast-DDS to use UDP transport instead of shared memory
# to avoid SHM transport errors that cause RViz display issues

echo "Setting up DDS transport configuration..."

# Set environment variables for Fast-DDS UDP transport (no SHM)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1

# Explicitly disable shared memory
export RMW_FASTRTPS_USE_SHARED_MEMORY=0

# Alternative: Use Cyclone DDS (more stable for some systems)
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "DDS transport configured to use UDP instead of shared memory"
echo "This should fix RViz image display issues"
