#!/usr/bin/env bash
# Shim so /opt/ros/humble/setup.bash doesn't fail when trying to source this
WS_DIR="/home/group7/Desktop/Autonomous-Drone/ros2_ws"

if [ -f "${WS_DIR}/install/setup.bash" ]; then
  . "${WS_DIR}/install/setup.bash"
fi
