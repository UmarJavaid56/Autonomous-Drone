# NED Pose Data Publisher

This script publishes pre-generated NED (North-East-Down) pose data for testing the MAVLink bridge with PX4 SITL.

## Usage

### Option 1: Simple Trajectories

```bash
# Circular trajectory
python3 publish_ned_pose.py --trajectory circle

# Straight line
python3 publish_ned_pose.py --trajectory line

# Hover in place
python3 publish_ned_pose.py --trajectory hover
```

### Option 2: Load from CSV File

```bash
# Publish poses from CSV file
python3 publish_ned_pose.py --csv sample_ned_poses.csv
```

CSV format: `x,y,z,qx,qy,qz,qw` (one pose per line)
- x, y, z: Position in NED frame (z is negative for altitude above ground)
- qx, qy, qz, qw: Quaternion orientation

### Option 3: Static Pose

```bash
# Publish a single static pose
python3 publish_ned_pose.py --x 1.0 --y 2.0 --z -2.0
```

### Additional Options

```bash
# Change publishing topic (default: /orbslam3/pose)
python3 publish_ned_pose.py --trajectory circle --topic /custom/pose

# Change publishing rate (default: 30 Hz)
python3 publish_ned_pose.py --trajectory circle --rate 10.0
```

## Testing with SITL

1. Start PX4 SITL:
   ```bash
   make px4_sitl_default jmavsim
   ```

2. Launch MAVROS and bridge:
   ```bash
   ros2 launch vio_bringup vio_sitl_mavros.launch.py
   ```

3. In another terminal, publish test poses:
   ```bash
   cd ros2_ws/src/vio_mavlink_bridge/scripts
   python3 publish_ned_pose.py --trajectory circle
   ```

## NED Frame Convention

- **X**: North (positive = north)
- **Y**: East (positive = east)
- **Z**: Down (positive = down, negative = up)
  - For altitude above ground, use negative z values
  - Example: z = -2.0 means 2 meters above ground

## Notes

- The script publishes `geometry_msgs/PoseStamped` messages
- Default topic is `/orbslam3/pose` (matches the bridge node's subscription)
- Timestamps are automatically set to current ROS time
- Frame ID is set to "world"

