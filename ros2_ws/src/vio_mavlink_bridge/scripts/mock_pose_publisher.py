#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import time

class MockPosePublisher(Node):
    def __init__(self):
        super().__init__('mock_pose_publisher')

        # Publisher for pose data (simulating ORB-SLAM3 output)
        self.pose_publisher = self.create_publisher(PoseStamped, '/orbslam3/pose', 10)

        # Timer to publish poses at 30 Hz
        self.timer = self.create_timer(1.0/30.0, self.publish_pose)

        # Simple circular trajectory
        self.start_time = time.time()
        self.radius = 2.0  # 2 meter radius
        self.height = -2.0  # 2 meters above ground (NED frame)

        self.get_logger().info('Mock pose publisher started - simulating camera SLAM output')

    def publish_pose(self):
        # Create pose message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'

        # Calculate position on circle
        elapsed = time.time() - self.start_time
        angle = elapsed * 0.2  # Slow rotation

        x = self.radius * math.cos(angle)
        y = self.radius * math.sin(angle)
        z = self.height

        # Set position
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z

        # Set orientation (facing center of circle)
        # Simple rotation around Z axis
        qw = math.cos(angle/2)
        qz = math.sin(angle/2)

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        # Publish pose
        self.pose_publisher.publish(pose_msg)

        self.get_logger().debug('.3f')

def main(args=None):
    rclpy.init(args=args)
    node = MockPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
