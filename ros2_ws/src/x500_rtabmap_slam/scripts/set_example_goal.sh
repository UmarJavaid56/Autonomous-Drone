#!/usr/bin/env bash
# Publish one example goal to /goal_pose so you can see RRT* path planning in RViz.
# Usage: ./set_example_goal.sh [x] [y] [z]
# Default: (2.0, 1.0, 1.0) in map frame.

X=${1:-2.0}
Y=${2:-1.0}
Z=${3:-1.0}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"
source "$WS/install/setup.bash" 2>/dev/null || true

ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: { frame_id: 'map' },
  pose: {
    position: { x: $X, y: $Y, z: $Z },
    orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
  }
}"

echo "Published goal at ($X, $Y, $Z). Check RViz for the path; press T in teleop to release control and let the drone follow."
