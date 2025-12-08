#!/usr/bin/env python3
"""
Script to publish pre-generated NED pose data for SITL testing.

Usage examples:
    # Publish a simple circular trajectory
    python3 publish_ned_pose.py --trajectory circle

    # Publish a straight line
    python3 publish_ned_pose.py --trajectory line

    # Load poses from CSV file
    python3 publish_ned_pose.py --csv poses.csv

    # Publish a single static pose
    python3 publish_ned_pose.py --x 1.0 --y 2.0 --z -0.5
"""

import rclpy
from rclpy.node import Node
import math
import csv
import argparse
import sys
from geometry_msgs.msg import PoseStamped


class NEDPosePublisher(Node):
    def __init__(self, trajectory_type=None, csv_file=None, static_pose=None, topic='/orbslam3/pose', rate=30.0):
        super().__init__('ned_pose_publisher')
        
        self.topic = topic
        self.rate = rate
        self.publisher_ = self.create_publisher(PoseStamped, self.topic, 10)
        
        # Trajectory state
        self.trajectory_type = trajectory_type
        self.csv_file = csv_file
        self.static_pose = static_pose
        self.csv_data = []
        self.csv_index = 0
        self.time_start = self.get_clock().now()
        
        if csv_file:
            self.load_csv(csv_file)
        
        # Create timer for publishing
        timer_period = 1.0 / rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f"Publishing NED poses to '{self.topic}' at {rate} Hz")
        if trajectory_type:
            self.get_logger().info(f"Trajectory type: {trajectory_type}")
        elif csv_file:
            self.get_logger().info(f"Loaded {len(self.csv_data)} poses from CSV")
        elif static_pose:
            self.get_logger().info(f"Static pose: {static_pose}")
    
    def load_csv(self, filename):
        """Load poses from CSV file. Expected format: x,y,z,qx,qy,qz,qw"""
        try:
            with open(filename, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) >= 3:
                        pose = {
                            'x': float(row[0]),
                            'y': float(row[1]),
                            'z': float(row[2]),
                            'qx': float(row[3]) if len(row) > 3 else 0.0,
                            'qy': float(row[4]) if len(row) > 4 else 0.0,
                            'qz': float(row[5]) if len(row) > 5 else 0.0,
                            'qw': float(row[6]) if len(row) > 6 else 1.0,
                        }
                        self.csv_data.append(pose)
        except Exception as e:
            self.get_logger().error(f"Failed to load CSV: {e}")
            sys.exit(1)
    
    def get_pose_from_trajectory(self, t):
        """Generate pose based on trajectory type"""
        if self.trajectory_type == 'circle':
            # Circular trajectory in NED frame
            radius = 5.0  # meters
            height = -2.0  # meters (negative = down in NED)
            x = radius * math.cos(t)
            y = radius * math.sin(t)
            z = height
            # Orientation: yaw follows the circle
            yaw = t + math.pi/2
            qx, qy, qz, qw = self.euler_to_quaternion(0, 0, yaw)
            return {'x': x, 'y': y, 'z': z, 'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw}
        
        elif self.trajectory_type == 'line':
            # Straight line trajectory
            speed = 2.0  # m/s
            x = speed * t
            y = 0.0
            z = -2.0
            yaw = 0.0
            qx, qy, qz, qw = self.euler_to_quaternion(0, 0, yaw)
            return {'x': x, 'y': y, 'z': z, 'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw}
        
        elif self.trajectory_type == 'hover':
            # Hover in place
            return {'x': 0.0, 'y': 0.0, 'z': -2.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
        
        else:
            return {'x': 0.0, 'y': 0.0, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw
    
    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        
        # Get pose based on source
        if self.static_pose:
            pose = self.static_pose
        elif self.csv_file and self.csv_data:
            # Use CSV data
            if self.csv_index < len(self.csv_data):
                pose = self.csv_data[self.csv_index]
                self.csv_index += 1
            else:
                # Loop back to start
                self.csv_index = 0
                pose = self.csv_data[0]
        else:
            # Use trajectory
            elapsed = (self.get_clock().now() - self.time_start).nanoseconds / 1e9
            pose = self.get_pose_from_trajectory(elapsed)
        
        # Fill message
        msg.pose.position.x = pose['x']
        msg.pose.position.y = pose['y']
        msg.pose.position.z = pose['z']
        msg.pose.orientation.x = pose['qx']
        msg.pose.orientation.y = pose['qy']
        msg.pose.orientation.z = pose['qz']
        msg.pose.orientation.w = pose['qw']
        
        self.publisher_.publish(msg)
        self.get_logger().debug(
            f"Published pose: pos=[{pose['x']:.3f}, {pose['y']:.3f}, {pose['z']:.3f}] "
            f"ori=[{pose['qx']:.3f}, {pose['qy']:.3f}, {pose['qz']:.3f}, {pose['qw']:.3f}]"
        )


def main(args=None):
    parser = argparse.ArgumentParser(description='Publish NED pose data for SITL testing')
    parser.add_argument('--trajectory', choices=['circle', 'line', 'hover'],
                       help='Trajectory type to generate')
    parser.add_argument('--csv', type=str, help='CSV file with pose data (x,y,z,qx,qy,qz,qw)')
    parser.add_argument('--x', type=float, help='Static X position (NED frame)')
    parser.add_argument('--y', type=float, help='Static Y position (NED frame)')
    parser.add_argument('--z', type=float, help='Static Z position (NED frame, negative = down)')
    parser.add_argument('--topic', type=str, default='/orbslam3/pose',
                       help='Topic to publish to (default: /orbslam3/pose)')
    parser.add_argument('--rate', type=float, default=30.0,
                       help='Publishing rate in Hz (default: 30.0)')
    
    args = parser.parse_args()
    
    # Determine pose source
    static_pose = None
    if args.x is not None or args.y is not None or args.z is not None:
        static_pose = {
            'x': args.x if args.x is not None else 0.0,
            'y': args.y if args.y is not None else 0.0,
            'z': args.z if args.z is not None else -2.0,
            'qx': 0.0,
            'qy': 0.0,
            'qz': 0.0,
            'qw': 1.0
        }
    
    if not args.trajectory and not args.csv and not static_pose:
        parser.print_help()
        sys.exit(1)
    
    rclpy.init(args=args)
    node = NEDPosePublisher(
        trajectory_type=args.trajectory,
        csv_file=args.csv,
        static_pose=static_pose,
        topic=args.topic,
        rate=args.rate
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

