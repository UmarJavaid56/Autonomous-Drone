#!/usr/bin/env python3
"""Path executor: follows nav_msgs/Path with velocity setpoints.

Subscribes to:
  - path (nav_msgs/Path): waypoints in map frame
  - planning_active (std_msgs/Bool): when true, do not advance waypoints (planning in progress)
  - TF map -> base_link for current pose

Publishes:
  - cmd_vel (geometry_msgs/Twist): velocity commands for simulation or PX4
  - enable (std_msgs/Bool): controller enable (true when following path)

Safety: if path is empty/stale, command hard-stop (zero velocity) by default.
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
        self.declare_parameter("hover_reference_frame_id", "odom")
        self.declare_parameter("path_topic", "path")
        self.declare_parameter("cmd_vel_topic", "/x500_depth/cmd_vel")
        self.declare_parameter("enable_topic", "/x500_depth/enable")
        self.declare_parameter("manual_override_topic", "/x500_depth/teleop_active")
        self.declare_parameter("waypoint_tolerance", 0.25)
        self.declare_parameter("max_linear_speed", 0.8)
        self.declare_parameter("max_angular_speed", 0.8)
        self.declare_parameter("linear_kp", 2.0)
        self.declare_parameter("min_linear_speed", 0.25)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("hover_on_no_path", True)
        self.declare_parameter("hard_stop_on_no_path", True)
        self.declare_parameter("airborne_height", 0.15)
        self.declare_parameter("rotate_to_heading_before_move", True)
        self.declare_parameter("heading_align_threshold_rad", 0.8)
        self.declare_parameter("min_xy_dist_for_heading_align", 0.4)
        self.declare_parameter("heading_hard_stop_threshold_rad", 2.6)
        self.declare_parameter("min_heading_speed_factor", 0.45)
        self.declare_parameter("cmd_vel_is_body_frame", True)
        self.declare_parameter("cmd_vel_body_x_gain", 1.0)
        self.declare_parameter("cmd_vel_body_y_gain", 1.0)
        self.declare_parameter("enforce_takeoff_before_xy", True)
        self.declare_parameter("min_altitude_for_xy_motion", 0.9)
        self.declare_parameter("enforce_min_target_altitude", True)
        self.declare_parameter("min_target_altitude", 0.9)
        self.declare_parameter("takeoff_altitude_tolerance", 0.12)
        self.declare_parameter("takeoff_vertical_speed", 0.6)
        self.declare_parameter("takeoff_vertical_kp", 1.2)
        self.declare_parameter("max_path_age_sec", 1.5)
        self.declare_parameter("min_consecutive_nonempty_paths", 3)
        self.declare_parameter("hover_brake_kp", 4.0)
        self.declare_parameter("hover_brake_max_speed", 1.5)
        self.declare_parameter("hover_hold_kp", 2.0)
        self.declare_parameter("hover_hold_max_speed", 1.5)
        self.declare_parameter("hover_total_max_speed", 1.5)
        self.declare_parameter("hard_stop_drift_guard", True)
        self.declare_parameter("hard_stop_drift_guard_pos_threshold", 0.02)
        self.declare_parameter("hard_stop_drift_guard_vel_threshold", 0.02)
        self.declare_parameter("hard_stop_drift_guard_hold_kp", 0.6)
        self.declare_parameter("hard_stop_drift_guard_brake_kp", 1.2)
        self.declare_parameter("hard_stop_drift_guard_max_speed", 0.20)

        self.map_frame_id = self.get_parameter("map_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.hover_reference_frame_id = self.get_parameter("hover_reference_frame_id").value
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
        self.hard_stop_on_no_path = self.get_parameter("hard_stop_on_no_path").value
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
        self.cmd_vel_body_x_gain = (
            self.get_parameter("cmd_vel_body_x_gain").value
        )
        self.cmd_vel_body_y_gain = (
            self.get_parameter("cmd_vel_body_y_gain").value
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
        self.max_path_age_sec = self.get_parameter("max_path_age_sec").value
        self.min_consecutive_nonempty_paths = int(
            self.get_parameter("min_consecutive_nonempty_paths").value
        )
        self.hover_brake_kp = self.get_parameter("hover_brake_kp").value
        self.hover_brake_max_speed = self.get_parameter("hover_brake_max_speed").value
        self.hover_hold_kp = self.get_parameter("hover_hold_kp").value
        self.hover_hold_max_speed = self.get_parameter("hover_hold_max_speed").value
        self.hover_total_max_speed = self.get_parameter("hover_total_max_speed").value
        self.hard_stop_drift_guard = (
            self.get_parameter("hard_stop_drift_guard").value
        )
        self.hard_stop_drift_guard_pos_threshold = (
            self.get_parameter("hard_stop_drift_guard_pos_threshold").value
        )
        self.hard_stop_drift_guard_vel_threshold = (
            self.get_parameter("hard_stop_drift_guard_vel_threshold").value
        )
        self.hard_stop_drift_guard_hold_kp = (
            self.get_parameter("hard_stop_drift_guard_hold_kp").value
        )
        self.hard_stop_drift_guard_brake_kp = (
            self.get_parameter("hard_stop_drift_guard_brake_kp").value
        )
        self.hard_stop_drift_guard_max_speed = (
            self.get_parameter("hard_stop_drift_guard_max_speed").value
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
        self.last_path_update_time = None
        self.consecutive_nonempty_paths = 0
        self.prev_pose_for_brake: Optional[tuple] = None
        self.prev_pose_time = None
        self.vel_est_map = (0.0, 0.0, 0.0)
        self.hover_anchor_xy: Optional[tuple] = None
        self.hard_stop_anchor_xy: Optional[tuple] = None

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
        self.last_path_update_time = self.get_clock().now()
        if len(msg.poses) < 2:
            self.current_path = None
            self.waypoint_index = 0
            self.consecutive_nonempty_paths = 0
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
        self.consecutive_nonempty_paths += 1
        self.waypoint_index = min(new_start_idx, len(msg.poses) - 1)

    def planning_active_callback(self, msg: Bool):
        self.planning_active = msg.data

    def manual_override_callback(self, msg: Bool):
        self.manual_override = msg.data

    def get_pose(self, target_frame_id: str, update_last_pose: bool = False) -> Optional[tuple]:
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame_id,
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
            if update_last_pose:
                self.last_pose = pose
            return pose
        except TransformException:
            return None

    def update_velocity_estimate(self, pose: Optional[tuple]):
        now = self.get_clock().now()
        if pose is None:
            vx, vy, vz = self.vel_est_map
            self.vel_est_map = (0.8 * vx, 0.8 * vy, 0.8 * vz)
            return

        if self.prev_pose_for_brake is not None and self.prev_pose_time is not None:
            dt = (now - self.prev_pose_time).nanoseconds * 1e-9
            if dt > 1e-3:
                x, y, z, _ = pose
                px, py, pz, _ = self.prev_pose_for_brake
                self.vel_est_map = ((x - px) / dt, (y - py) / dt, (z - pz) / dt)

        self.prev_pose_for_brake = pose
        self.prev_pose_time = now

    def publish_hover(self, pose: Optional[tuple], reason: Optional[str] = None):
        twist = Twist()
        enable = Bool()
        enable.data = True

        hold_x = 0.0
        hold_y = 0.0
        if pose is not None:
            x, y, _, _ = pose
            if self.hover_anchor_xy is None:
                self.hover_anchor_xy = (x, y)
            ax, ay = self.hover_anchor_xy
            hold_x = self.hover_hold_kp * (ax - x)
            hold_y = self.hover_hold_kp * (ay - y)
            hold_speed = math.sqrt(hold_x * hold_x + hold_y * hold_y)
            if hold_speed > self.hover_hold_max_speed and hold_speed > 1e-6:
                s = self.hover_hold_max_speed / hold_speed
                hold_x *= s
                hold_y *= s

        vx, vy, _ = self.vel_est_map
        bx_map = -self.hover_brake_kp * vx
        by_map = -self.hover_brake_kp * vy
        b_speed = math.sqrt(bx_map * bx_map + by_map * by_map)
        if b_speed > self.hover_brake_max_speed and b_speed > 1e-6:
            scale = self.hover_brake_max_speed / b_speed
            bx_map *= scale
            by_map *= scale

        cmd_x_map = hold_x + bx_map
        cmd_y_map = hold_y + by_map
        cmd_speed = math.sqrt(cmd_x_map * cmd_x_map + cmd_y_map * cmd_y_map)
        if cmd_speed > self.hover_total_max_speed and cmd_speed > 1e-6:
            s = self.hover_total_max_speed / cmd_speed
            cmd_x_map *= s
            cmd_y_map *= s

        if self.cmd_vel_is_body_frame and pose is not None:
            yaw = pose[3]
            cy = math.cos(yaw)
            sy = math.sin(yaw)
            twist.linear.x = float(self.cmd_vel_body_x_gain * (cy * cmd_x_map + sy * cmd_y_map))
            twist.linear.y = float(self.cmd_vel_body_y_gain * (-sy * cmd_x_map + cy * cmd_y_map))
        else:
            twist.linear.x = float(cmd_x_map)
            twist.linear.y = float(cmd_y_map)
        twist.linear.z = 0.0
        twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)
        self.enable_pub.publish(enable)
        if reason is not None:
            self.get_logger().warn(reason, throttle_duration_sec=1.0)

    def publish_stop(self, reason: Optional[str] = None):
        twist = Twist()
        enable = Bool()
        enable.data = True
        self.cmd_vel_pub.publish(twist)
        self.enable_pub.publish(enable)
        if reason is not None:
            self.get_logger().warn(reason, throttle_duration_sec=1.0)

    def publish_no_path_command(self, pose: Optional[tuple], reason: Optional[str] = None):
        if self.hard_stop_on_no_path:
            if self.hard_stop_drift_guard and pose is not None:
                x, y, _, _ = pose
                if self.hard_stop_anchor_xy is None:
                    self.hard_stop_anchor_xy = (x, y)
                ax, ay = self.hard_stop_anchor_xy
                ex = ax - x
                ey = ay - y
                vx, vy, _ = self.vel_est_map
                pos_err = math.hypot(ex, ey)
                vel_mag = math.hypot(vx, vy)

                if (
                    pos_err > self.hard_stop_drift_guard_pos_threshold
                    or vel_mag > self.hard_stop_drift_guard_vel_threshold
                ):
                    cmd_x_map = self.hard_stop_drift_guard_hold_kp * ex - self.hard_stop_drift_guard_brake_kp * vx
                    cmd_y_map = self.hard_stop_drift_guard_hold_kp * ey - self.hard_stop_drift_guard_brake_kp * vy
                    cmd_mag = math.hypot(cmd_x_map, cmd_y_map)
                    cap = max(
                        0.0,
                        min(self.hover_total_max_speed, self.hard_stop_drift_guard_max_speed),
                    )
                    if cmd_mag > cap and cmd_mag > 1e-6:
                        s = cap / cmd_mag
                        cmd_x_map *= s
                        cmd_y_map *= s

                    twist = Twist()
                    if self.cmd_vel_is_body_frame:
                        yaw = pose[3]
                        cy = math.cos(yaw)
                        sy = math.sin(yaw)
                        twist.linear.x = float(
                            self.cmd_vel_body_x_gain * (cy * cmd_x_map + sy * cmd_y_map)
                        )
                        twist.linear.y = float(
                            self.cmd_vel_body_y_gain * (-sy * cmd_x_map + cy * cmd_y_map)
                        )
                    else:
                        twist.linear.x = float(cmd_x_map)
                        twist.linear.y = float(cmd_y_map)
                    twist.linear.z = 0.0
                    twist.angular.z = 0.0

                    enable = Bool()
                    enable.data = True
                    self.cmd_vel_pub.publish(twist)
                    self.enable_pub.publish(enable)
                    if reason is not None:
                        self.get_logger().warn(
                            (
                                "%s | hard-stop drift guard active: pos_err=%.3f vel=%.3f cmd_xy=(%.3f,%.3f)"
                                % (reason, pos_err, vel_mag, twist.linear.x, twist.linear.y)
                            ),
                            throttle_duration_sec=1.0,
                        )
                    return

            # Reset hover anchor so fallback hover mode doesn't jump later.
            self.hover_anchor_xy = None
            self.publish_stop(reason)
        else:
            self.hard_stop_anchor_xy = None
            self.publish_hover(pose, reason)

    def control_callback(self):
        if self.manual_override:
            return

        pose_hover = self.get_pose(self.hover_reference_frame_id)
        self.update_velocity_estimate(pose_hover)

        if not self.path_received:
            # Publish explicit stop until the planner has produced at least one path.
            # This avoids carrying any stale cmd_vel from previous runs.
            self.publish_no_path_command(pose_hover)
            return

        twist = Twist()
        enable = Bool()

        # Safety: hover if no path and keep controller enabled.
        if self.current_path is None or len(self.current_path.poses) < 2:
            self.publish_no_path_command(pose_hover, "no path: hard stop")
            return

        # Require frequent path refresh. If planner output stalls for any reason,
        # fail safe to stop instead of following stale waypoints.
        if self.last_path_update_time is None:
            self.publish_no_path_command(pose_hover, "missing path timestamp: hard stop")
            return
        path_age = (self.get_clock().now() - self.last_path_update_time).nanoseconds * 1e-9
        if path_age > self.max_path_age_sec:
            self.publish_no_path_command(
                pose_hover,
                "stale path (age=%.2fs > %.2fs): hard stop"
                % (path_age, self.max_path_age_sec),
            )
            return

        if self.consecutive_nonempty_paths < self.min_consecutive_nonempty_paths:
            self.publish_no_path_command(
                pose_hover,
                "path not stable yet (%d/%d non-empty updates): hard stop"
                % (self.consecutive_nonempty_paths, self.min_consecutive_nonempty_paths),
            )
            return

        # Follow the latest path even while planner is recomputing; the planner
        # replans continuously (2 Hz) so gating on planning_active would block
        # execution permanently.
        self.hover_anchor_xy = None
        self.hard_stop_anchor_xy = None
        pose_map = self.get_pose(self.map_frame_id, update_last_pose=True)
        if pose_map is None:
            # Keep controller enabled on brief TF loss so motors don't drop out.
            self.publish_no_path_command(pose_hover, "map TF unavailable: hard stop")
            return

        x, y, z, yaw = pose_map
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
                twist.linear.x = float(self.cmd_vel_body_x_gain * (cy * vx_map + sy * vy_map))
                twist.linear.y = float(self.cmd_vel_body_y_gain * (-sy * vx_map + cy * vy_map))
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
