#!/usr/bin/env python3
"""Convert RViz 'Publish Point' clicks to goal_pose for the RRT* planner.

Subscribes to /clicked_point (geometry_msgs/PointStamped) from RViz's
'Publish Point' tool and publishes geometry_msgs/PoseStamped to /goal_pose
with identity orientation (yaw=0). Lets you click a 3D point in the scene
to set the navigation goal.

You can also use the '2D Goal Pose' tool in RViz (topic /goal_pose) to set
a goal by click-and-drag (x, y, yaw; z may be 0).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped


class GoalFromRvizNode(Node):
    def __init__(self):
        super().__init__("goal_from_rviz")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("default_yaw", 0.0)

        clicked_topic = self.get_parameter("clicked_point_topic").value
        goal_topic = self.get_parameter("goal_pose_topic").value

        self.sub = self.create_subscription(
            PointStamped,
            clicked_topic,
            self.cb_clicked_point,
            10,
        )
        self.pub = self.create_publisher(PoseStamped, goal_topic, 10)

        self.get_logger().info(
            f"Goal from RViz: {clicked_topic} -> {goal_topic} (click a 3D point to set goal)"
        )

    def cb_clicked_point(self, msg: PointStamped):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.point
        # Identity orientation (yaw=0)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        self.pub.publish(pose)
        self.get_logger().info(
            f"Goal set from click: ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f}) "
            f"frame={msg.header.frame_id}"
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
