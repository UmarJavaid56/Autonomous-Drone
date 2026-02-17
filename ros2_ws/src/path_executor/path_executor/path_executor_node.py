#!/usr/bin/env python3
"""Path executor: follows nav_msgs/Path with velocity setpoints; hover on empty path or planning failure.

Subscribes to:
  - path (nav_msgs/Path): waypoints in map frame
  - planning_active (std_msgs/Bool): when true, do not advance waypoints (planning in progress)
  - TF map -> base_link for current pose

Publishes:
  - cmd_vel (geometry_msgs/Twist): velocity commands for simulation or PX4
  - enable (std_msgs/Bool): controller enable (true when following path)

Safety: if path is empty or planning_active and path is stale, command hover (zero velocity).
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


class PathExecutorNode(Node):
    def __init__(self):
        super().__init__("path_executor")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("path_topic", "path")
        self.declare_parameter("cmd_vel_topic", "/x500_depth/cmd_vel")
        self.declare_parameter("enable_topic", "/x500_depth/enable")
        self.declare_parameter("manual_override_topic", "/x500_depth/teleop_active")
        self.declare_parameter("waypoint_tolerance", 0.25)
        self.declare_parameter("max_linear_speed", 0.8)
        self.declare_parameter("max_angular_speed", 0.5)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("hover_on_no_path", True)
        self.declare_parameter("airborne_height", 0.15)

        self.map_frame_id = self.get_parameter("map_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        path_topic = self.get_parameter("path_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        enable_topic = self.get_parameter("enable_topic").value
        manual_override_topic = self.get_parameter("manual_override_topic").value
        self.waypoint_tolerance = self.get_parameter("waypoint_tolerance").value
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value
        self.hover_on_no_path = self.get_parameter("hover_on_no_path").value
        self.airborne_height = self.get_parameter("airborne_height").value
        control_rate = self.get_parameter("control_rate").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_sub = self.create_subscription(
            Path, path_topic, self.path_callback, 10
        )
        self.planning_active_sub = self.create_subscription(
            Bool, "planning_active", self.planning_active_callback, 10
        )
        self.manual_override_sub = self.create_subscription(
            Bool, manual_override_topic, self.manual_override_callback, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.enable_pub = self.create_publisher(Bool, enable_topic, 10)

        self.current_path: Optional[Path] = None
        self.waypoint_index = 0
        self.planning_active = False
        self.path_received = False
        self.manual_override = False
        self.last_pose: Optional[tuple] = None

        self.control_timer = self.create_timer(
            1.0 / control_rate, self.control_callback
        )

        self.get_logger().info(
            "Path executor: path={}, cmd_vel={}, enable={}, manual_override={}".format(
                path_topic, cmd_vel_topic, enable_topic, manual_override_topic
            )
        )

    def path_callback(self, msg: Path):
        if len(msg.poses) < 2:
            self.current_path = None
            self.waypoint_index = 0
            return
        self.current_path = msg
        self.path_received = True
        # Optionally reset waypoint index when path is updated (replan)
        self.waypoint_index = 0

    def planning_active_callback(self, msg: Bool):
        self.planning_active = msg.data

    def manual_override_callback(self, msg: Bool):
        self.manual_override = msg.data

    def get_current_pose(self) -> Optional[tuple]:
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.base_frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            q = t.transform.rotation
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            pose = (x, y, z, yaw)
            self.last_pose = pose
            return pose
        except TransformException:
            return None

    def control_callback(self):
        if self.manual_override:
            return

        twist = Twist()
        enable = Bool()

        # Safety: hover if no path (keep controller enabled so drone doesn't drop)
        if self.current_path is None or len(self.current_path.poses) < 2:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.z = 0.0
            # Only disable when we're clearly on the ground; otherwise hover (enable=True)
            if (
                self.hover_on_no_path
                and self.last_pose is not None
                and self.last_pose[2] < self.airborne_height
            ):
                enable.data = False
            else:
                enable.data = True
            self.cmd_vel_pub.publish(twist)
            self.enable_pub.publish(enable)
            return

        if self.planning_active:
            # Hover while planner is running
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.z = 0.0
            enable.data = True
            self.cmd_vel_pub.publish(twist)
            self.enable_pub.publish(enable)
            return

        pose = self.get_current_pose()
        if pose is None:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.z = 0.0
            # Keep hover (enable=True) unless clearly on ground; avoids drop on brief TF loss
            if (
                self.hover_on_no_path
                and self.last_pose is not None
                and self.last_pose[2] < self.airborne_height
            ):
                enable.data = False
            else:
                enable.data = True
            self.cmd_vel_pub.publish(twist)
            self.enable_pub.publish(enable)
            return

        x, y, z, yaw = pose
        target = self.current_path.poses[self.waypoint_index].pose.position
        tx, ty, tz = target.x, target.y, target.z

        dx = tx - x
        dy = ty - y
        dz = tz - z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < self.waypoint_tolerance:
            self.waypoint_index = min(
                self.waypoint_index + 1, len(self.current_path.poses) - 1
            )
            target = self.current_path.poses[self.waypoint_index].pose.position
            tx, ty, tz = target.x, target.y, target.z
            dx = tx - x
            dy = ty - y
            dz = tz - z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < 1e-6:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.z = 0.0
        else:
            # Simple P: proportional velocity toward target
            scale = self.max_linear_speed / max(dist, 0.1)
            scale = min(scale, 1.0)
            twist.linear.x = float(dx * scale)
            twist.linear.y = float(dy * scale)
            twist.linear.z = float(dz * scale)
            # Yaw toward next waypoint (optional)
            desired_yaw = math.atan2(dy, dx)
            yaw_err = desired_yaw - yaw
            while yaw_err > math.pi:
                yaw_err -= 2 * math.pi
            while yaw_err < -math.pi:
                yaw_err += 2 * math.pi
            twist.angular.z = float(
                max(-self.max_angular_speed, min(self.max_angular_speed, yaw_err * 2.0))
            )

        enable.data = True
        self.cmd_vel_pub.publish(twist)
        self.enable_pub.publish(enable)


def main(args=None):
    rclpy.init(args=args)
    node = PathExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
