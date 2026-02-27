#!/usr/bin/env python3
"""Sim-time-aware takeoff node.

Waits for the sim clock, enables the velocity controller, commands a vertical
ascent for a configurable duration (in sim time), then publishes one hover
command and stops so the path executor becomes the sole velocity publisher.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class TakeoffNode(Node):
    def __init__(self):
        super().__init__("takeoff_node")
        self.declare_parameter("takeoff_speed", 0.6)
        self.declare_parameter("takeoff_duration", 3.0)
        self.declare_parameter("cmd_vel_topic", "/x500_depth/cmd_vel")
        self.declare_parameter("enable_topic", "/x500_depth/enable")
        self.declare_parameter("publish_rate", 20.0)

        self.takeoff_speed = self.get_parameter("takeoff_speed").value
        self.takeoff_duration = self.get_parameter("takeoff_duration").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        enable_topic = self.get_parameter("enable_topic").value
        rate = self.get_parameter("publish_rate").value

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.enable_pub = self.create_publisher(Bool, enable_topic, 10)

        self.enabled = False
        self.takeoff_start_time = None
        self.phase = "enable"  # enable -> ascend -> done

        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().info(
            f"Takeoff node: speed={self.takeoff_speed}, "
            f"duration={self.takeoff_duration}s (sim time)"
        )

    def timer_callback(self):
        now = self.get_clock().now()

        if self.phase == "enable":
            enable_msg = Bool()
            enable_msg.data = True
            self.enable_pub.publish(enable_msg)
            self.enabled = True
            self.takeoff_start_time = now
            self.phase = "ascend"
            self.get_logger().info("Controller enabled, starting ascent")
            return

        elapsed = (now - self.takeoff_start_time).nanoseconds / 1e9

        twist = Twist()
        if self.phase == "ascend":
            if elapsed < self.takeoff_duration:
                twist.linear.z = self.takeoff_speed
            else:
                self.phase = "done"
                self.get_logger().info(
                    f"Ascent complete after {elapsed:.1f}s sim time, handing control to path executor"
                )
                # Publish one zero-velocity command, then stop publishing to avoid
                # fighting with path_executor on /cmd_vel.
                self.cmd_vel_pub.publish(twist)
                self.timer.cancel()
                return

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
