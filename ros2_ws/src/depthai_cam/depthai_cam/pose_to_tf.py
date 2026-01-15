#!/usr/bin/env python3
"""Node to convert Gazebo pose to TF transform for odom->base_link.

Gazebo Harmonic publishes gz.msgs.Pose_V which bridges to PoseArray.
This node extracts the model pose and publishes odom->base_link TF.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from tf2_ros import TransformBroadcaster


class PoseToTFNode(Node):
    def __init__(self):
        super().__init__('pose_to_tf_node')
        
        # Declare parameters
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('pose_topic', '/model/x500_depth/pose')
        self.declare_parameter('use_pose_array', True)  # Gazebo publishes Pose_V -> PoseArray
        
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        pose_topic = self.get_parameter('pose_topic').value
        self.use_pose_array = self.get_parameter('use_pose_array').value
        
        self.get_logger().info(f'Publishing TF: {self.parent_frame} -> {self.child_frame}')
        self.get_logger().info(f'Subscribing to pose topic: {pose_topic}')
        self.get_logger().info(f'Using PoseArray: {self.use_pose_array}')
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to pose topic - Gazebo Pose_V bridges to PoseArray
        if self.use_pose_array:
            self.subscription = self.create_subscription(
                PoseArray,
                pose_topic,
                self.pose_array_callback,
                10
            )
        else:
            self.subscription = self.create_subscription(
                Pose,
                pose_topic,
                self.pose_callback,
                10
            )
        
        self.msg_received = False
    
    def pose_array_callback(self, msg: PoseArray):
        """Handle PoseArray from Gazebo (gz.msgs.Pose_V)."""
        if not self.msg_received:
            self.get_logger().info(f'First PoseArray received with {len(msg.poses)} poses')
            self.msg_received = True
        
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty PoseArray', throttle_duration_sec=5.0)
            return
        
        # First pose in the array is typically the model pose
        self.publish_transform(msg.poses[0], msg.header.stamp)
    
    def pose_callback(self, msg: Pose):
        """Handle single Pose message."""
        if not self.msg_received:
            self.get_logger().info('First Pose message received')
            self.msg_received = True
        
        self.publish_transform(msg, self.get_clock().now().to_msg())
    
    def publish_transform(self, pose: Pose, stamp):
        """Publish TF transform from pose."""
        t = TransformStamped()
        t.header.stamp = stamp if stamp.sec != 0 else self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        
        # Copy position
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        
        # Copy orientation
        t.transform.rotation = pose.orientation
        
        # Publish transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToTFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
