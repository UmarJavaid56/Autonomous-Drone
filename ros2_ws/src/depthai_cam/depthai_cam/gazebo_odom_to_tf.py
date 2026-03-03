#!/usr/bin/env python3
"""Publish odom -> base_link TF from bridged nav_msgs/Odometry.

The Gazebo OdometryPublisher publishes to /world/<world>/model/<name>/odom (gz.msgs.Odometry).
We bridge that to /odom; this node republishes the pose as a TF so that
odom -> base_link is available on /tf for RTAB-Map and the rest of the stack.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def main(args=None):
    rclpy.init(args=args)
    node = GazeboOdomToTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


class GazeboOdomToTfNode(Node):
    def __init__(self):
        super().__init__("gazebo_odom_to_tf")
        self.declare_parameter("odom_topic", "odom")
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_cb,
            10,
        )
        self.get_logger().info(
            "Publishing TF from %s (frame_id) -> base_link (child_frame_id)" % odom_topic
        )

    def odom_cb(self, msg):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


if __name__ == "__main__":
    main()
