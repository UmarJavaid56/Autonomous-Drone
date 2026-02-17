#!/usr/bin/env python3
"""Convert RViz 'Publish Point' and '2D Goal Pose' to goal_pose for the RRT* planner.

- Publish Point: subscribes to /clicked_point, publishes PoseStamped to /goal_pose_planned.
- 2D Goal Pose: RViz publishes to /goal_pose with z=0 (ground). We subscribe and when
  goal z looks like ground (z < threshold), overwrite z with current drone height so the
  goal is in the drone's flight plane. Output goes to /goal_pose_planned (separate topic
  to avoid feedback loop).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


class GoalFromRvizNode(Node):
    def __init__(self):
        super().__init__("goal_from_rviz")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("goal_pose_in_topic", "/goal_pose")
        self.declare_parameter("goal_pose_out_topic", "/goal_pose_planned")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("use_drone_height_for_2d_goal", True)
        self.declare_parameter("ground_z_threshold", 0.4)

        clicked_topic = self.get_parameter("clicked_point_topic").value
        goal_in_topic = self.get_parameter("goal_pose_in_topic").value
        goal_out_topic = self.get_parameter("goal_pose_out_topic").value
        self.map_frame_id = self.get_parameter("map_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.use_drone_height = self.get_parameter("use_drone_height_for_2d_goal").value
        self.ground_z_threshold = self.get_parameter("ground_z_threshold").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_clicked = self.create_subscription(
            PointStamped,
            clicked_topic,
            self.cb_clicked_point,
            10,
        )
        self.sub_goal = self.create_subscription(
            PoseStamped,
            goal_in_topic,
            self.cb_goal_pose,
            10,
        )
        self.pub = self.create_publisher(PoseStamped, goal_out_topic, 10)

        self.get_logger().info(
            f"Goal from RViz: {clicked_topic} & {goal_in_topic} -> {goal_out_topic} "
            f"(2D goal z corrected to drone height: {self.use_drone_height})"
        )

    def _get_drone_z_in_map(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.base_frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            return t.transform.translation.z
        except TransformException:
            return None

    def cb_clicked_point(self, msg: PointStamped):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.point
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        self.pub.publish(pose)
        self.get_logger().info(
            f"Goal set from click: ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f}) "
            f"frame={msg.header.frame_id}"
        )

    def cb_goal_pose(self, msg: PoseStamped):
        z = msg.pose.position.z
        if not self.use_drone_height or z > self.ground_z_threshold:
            self.pub.publish(msg)
            self.get_logger().info(
                "Goal forwarded as-is: (x=%.2f, y=%.2f, z=%.2f)"
                % (msg.pose.position.x, msg.pose.position.y, z)
            )
            return
        drone_z = self._get_drone_z_in_map()
        if drone_z is None:
            self.get_logger().warning(
                "2D goal received (z=%.2f) but could not get drone height from TF; forwarding as-is."
                % z,
                throttle_duration_sec=2.0,
            )
            self.pub.publish(msg)
            return
        msg.pose.position.z = float(drone_z)
        self.pub.publish(msg)
        self.get_logger().info(
            "Goal from 2D Goal Pose: (x=%.2f, y=%.2f) z corrected %.2f -> %.2f (drone height)"
            % (msg.pose.position.x, msg.pose.position.y, z, drone_z)
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoalFromRvizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
