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
        self.declare_parameter("linear_kp", 2.0)
        self.declare_parameter("min_linear_speed", 0.25)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("hover_on_no_path", True)
        self.declare_parameter("airborne_height", 0.15)
        self.declare_parameter("rotate_to_heading_before_move", True)
        self.declare_parameter("heading_align_threshold_rad", 0.35)
        self.declare_parameter("min_xy_dist_for_heading_align", 0.10)
        self.declare_parameter("heading_hard_stop_threshold_rad", 1.0)
        self.declare_parameter("min_heading_speed_factor", 0.2)
        self.declare_parameter("cmd_vel_is_body_frame", True)
        self.declare_parameter("enforce_takeoff_before_xy", True)
        self.declare_parameter("min_altitude_for_xy_motion", 0.9)
        self.declare_parameter("enforce_min_target_altitude", True)
        self.declare_parameter("min_target_altitude", 0.9)
        self.declare_parameter("takeoff_altitude_tolerance", 0.05)
        self.declare_parameter("takeoff_vertical_speed", 0.6)
        self.declare_parameter("takeoff_vertical_kp", 1.2)

        self.map_frame_id = self.get_parameter("map_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        path_topic = self.get_parameter("path_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        enable_topic = self.get_parameter("enable_topic").value
        manual_override_topic = self.get_parameter("manual_override_topic").value
        self.waypoint_tolerance = self.get_parameter("waypoint_tolerance").value
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value
        self.linear_kp = self.get_parameter("linear_kp").value
        self.min_linear_speed = self.get_parameter("min_linear_speed").value
        self.hover_on_no_path = self.get_parameter("hover_on_no_path").value
        self.airborne_height = self.get_parameter("airborne_height").value
        self.rotate_to_heading_before_move = (
            self.get_parameter("rotate_to_heading_before_move").value
        )
        self.heading_align_threshold_rad = (
            self.get_parameter("heading_align_threshold_rad").value
        )
        self.min_xy_dist_for_heading_align = (
            self.get_parameter("min_xy_dist_for_heading_align").value
        )
        self.heading_hard_stop_threshold_rad = (
            self.get_parameter("heading_hard_stop_threshold_rad").value
        )
        self.min_heading_speed_factor = (
            self.get_parameter("min_heading_speed_factor").value
        )
        self.cmd_vel_is_body_frame = (
            self.get_parameter("cmd_vel_is_body_frame").value
        )
        self.enforce_takeoff_before_xy = (
            self.get_parameter("enforce_takeoff_before_xy").value
        )
        self.min_altitude_for_xy_motion = (
            self.get_parameter("min_altitude_for_xy_motion").value
        )
        self.enforce_min_target_altitude = (
            self.get_parameter("enforce_min_target_altitude").value
        )
        self.min_target_altitude = (
            self.get_parameter("min_target_altitude").value
        )
        self.takeoff_altitude_tolerance = (
            self.get_parameter("takeoff_altitude_tolerance").value
        )
        self.takeoff_vertical_speed = (
            self.get_parameter("takeoff_vertical_speed").value
        )
        self.takeoff_vertical_kp = (
            self.get_parameter("takeoff_vertical_kp").value
        )
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
        # Mark that planner is alive even when it publishes an empty path.
        # This lets control_callback command hover + enable instead of idling.
        self.path_received = True
        if len(msg.poses) < 2:
            self.current_path = None
            self.waypoint_index = 0
            return

        # On replan: find the closest waypoint to current position so the drone
        # doesn't jump back to waypoint 0 (which is the start == current pos).
        # Skip waypoints the drone has already passed.
        new_start_idx = 1  # skip waypoint 0 (== drone position) by default
        if self.last_pose is not None and len(msg.poses) > 2:
            cx, cy, cz = self.last_pose[0], self.last_pose[1], self.last_pose[2]
            best_idx = 1
            best_dist = float("inf")
            for i in range(1, len(msg.poses)):
                p = msg.poses[i].pose.position
                d = math.sqrt(
                    (p.x - cx) ** 2 + (p.y - cy) ** 2 + (p.z - cz) ** 2
                )
                if d < best_dist:
                    best_dist = d
                    best_idx = i
            # Advance past the closest waypoint if we're already within tolerance
            if best_dist < self.waypoint_tolerance and best_idx + 1 < len(msg.poses):
                new_start_idx = best_idx + 1
            else:
                new_start_idx = best_idx

        self.current_path = msg
        self.waypoint_index = min(new_start_idx, len(msg.poses) - 1)

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

        if not self.path_received:
            return

        twist = Twist()
        enable = Bool()

        # Safety: hover if no path and keep controller enabled.
        if self.current_path is None or len(self.current_path.poses) < 2:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.z = 0.0
            enable.data = True
            self.cmd_vel_pub.publish(twist)
            self.enable_pub.publish(enable)
            return

        # Follow the latest path even while planner is recomputing; the planner
        # replans continuously (2 Hz) so gating on planning_active would block
        # execution permanently.

        pose = self.get_current_pose()
        if pose is None:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.z = 0.0
            # Keep controller enabled on brief TF loss so motors don't drop out.
            enable.data = True
            self.cmd_vel_pub.publish(twist)
            self.enable_pub.publish(enable)
            return

        x, y, z, yaw = pose
        if (
            self.enforce_takeoff_before_xy
            and z < (self.min_altitude_for_xy_motion - self.takeoff_altitude_tolerance)
        ):
            # Safety gate: climb before allowing horizontal motion.
            climb_err = self.min_altitude_for_xy_motion - z
            vz = max(0.15, self.takeoff_vertical_kp * climb_err)
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = float(min(self.takeoff_vertical_speed, vz))
            twist.angular.z = 0.0
            enable.data = True
            self.cmd_vel_pub.publish(twist)
            self.enable_pub.publish(enable)
            self.get_logger().info(
                "takeoff-gate: z=%.2f < %.2f, climb cmd_z=%.2f"
                % (z, self.min_altitude_for_xy_motion, twist.linear.z),
                throttle_duration_sec=1.0,
            )
            return

        target = self.current_path.poses[self.waypoint_index].pose.position
        tx, ty, tz = target.x, target.y, target.z
        if self.enforce_min_target_altitude:
            tz = max(tz, self.min_target_altitude)

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
            if self.enforce_min_target_altitude:
                tz = max(tz, self.min_target_altitude)
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

            # Use a true speed controller: speed = kp * distance, clamped.
            speed_cmd = min(self.max_linear_speed, self.linear_kp * dist)
            if dist > self.waypoint_tolerance:
                speed_cmd = max(speed_cmd, self.min_linear_speed)
            speed_cmd = max(0.0, speed_cmd)
            inv_dist = 1.0 / max(dist, 1e-6)
            vx_map = float(dx * inv_dist * speed_cmd)
            vy_map = float(dy * inv_dist * speed_cmd)
            vz_map = float(dz * inv_dist * speed_cmd)
            twist.linear.z = vz_map

            # Gazebo MulticopterVelocityControl expects linear velocity in body
            # frame. Convert map-frame tracking velocity to body frame.
            if self.cmd_vel_is_body_frame:
                cy = math.cos(yaw)
                sy = math.sin(yaw)
                twist.linear.x = float(cy * vx_map + sy * vy_map)
                twist.linear.y = float(-sy * vx_map + cy * vy_map)
            else:
                twist.linear.x = vx_map
                twist.linear.y = vy_map

            # Optional safety behavior: rotate in place to face the waypoint
            # before translating, reducing sideways flight into unseen obstacles.
            horiz_dist = math.sqrt(dx * dx + dy * dy)
            if (
                self.rotate_to_heading_before_move
                and horiz_dist > self.min_xy_dist_for_heading_align
                and abs(yaw_err) > self.heading_align_threshold_rad
            ):
                # Slow translation while turning. Only hard-stop for large yaw error.
                hard_stop = max(
                    self.heading_align_threshold_rad, self.heading_hard_stop_threshold_rad
                )
                if abs(yaw_err) >= hard_stop:
                    factor = 0.0
                else:
                    factor = 1.0 - (abs(yaw_err) / hard_stop)
                    factor = max(self.min_heading_speed_factor, factor)
                twist.linear.x *= factor
                twist.linear.y *= factor
                twist.linear.z *= factor

        enable.data = True
        self.cmd_vel_pub.publish(twist)
        self.enable_pub.publish(enable)

        self.get_logger().info(
            "wp %d/%d dist=%.2f cmd=(%.2f,%.2f,%.2f) pos=(%.2f,%.2f,%.2f) tgt=(%.2f,%.2f,%.2f)"
            % (
                self.waypoint_index,
                len(self.current_path.poses),
                dist,
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                x, y, z,
                tx, ty, tz,
            ),
            throttle_duration_sec=2.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = PathExecutorNode()
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
