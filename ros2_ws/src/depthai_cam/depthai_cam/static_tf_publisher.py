#!/usr/bin/env python3
"""
Static TF publisher for basic simulation frames.
Publishes world -> odom -> base_link transform chain.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_frame_publisher')
        
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish static transforms
        self.publish_static_transforms()
        
        self.get_logger().info('Static TF publisher initialized')

    def publish_static_transforms(self):
        static_transforms = []
        
        # world -> odom (identity transform)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        static_transforms.append(t)
        
        # odom -> base_link (identity transform - will be updated dynamically later)
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'base_link'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.5  # Spawn slightly above ground
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        static_transforms.append(t2)
        
        # base_link -> camera_link (fixed camera mount)
        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = 'base_link'
        t3.child_frame_id = 'camera_link'
        t3.transform.translation.x = 0.12
        t3.transform.translation.y = 0.03
        t3.transform.translation.z = 0.242
        t3.transform.rotation.x = 0.0
        t3.transform.rotation.y = 0.0
        t3.transform.rotation.z = 0.0
        t3.transform.rotation.w = 1.0
        static_transforms.append(t3)
        
        self.tf_static_broadcaster.sendTransform(static_transforms)


def main():
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
