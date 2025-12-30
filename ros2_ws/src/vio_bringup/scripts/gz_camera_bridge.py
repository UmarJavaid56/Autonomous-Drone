#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import json
import base64
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class GzCameraBridge(Node):
    def __init__(self):
        super().__init__('gz_camera_bridge')

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        # Gazebo camera topic
        self.gz_topic = '/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image'

        # Timer to poll Gazebo topic (not ideal but works)
        self.timer = self.create_timer(0.1, self.poll_camera)  # 10 Hz

        self.get_logger().info('Gazebo camera bridge started - polling camera images')

    def poll_camera(self):
        """Poll Gazebo camera topic and publish to ROS"""
        try:
            # Use gz topic echo to get camera data
            cmd = ['gz', 'topic', '-e', '-n', '1', self.gz_topic]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=1)

            if result.returncode == 0 and result.stdout.strip():
                # Parse the gz message output
                # This is a simplified approach - gz topic echo outputs JSON-like data
                lines = result.stdout.strip().split('\n')
                for line in lines:
                    if 'data:' in line and 'width:' in line:
                        # Extract image data (simplified parsing)
                        # This would need proper gz message parsing in a real implementation
                        pass

                # For now, create a dummy image to test the pipeline
                # In a real implementation, you'd parse the gz message properly
                dummy_image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(dummy_image, f"Gazebo Camera {self.get_clock().now().nanoseconds}",
                          (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                # Convert to ROS Image
                ros_image = self.bridge.cv2_to_imgmsg(dummy_image, encoding="bgr8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = "camera_link"

                self.publisher.publish(ros_image)
                self.get_logger().debug("Published dummy camera image")

        except subprocess.TimeoutExpired:
            self.get_logger().warn("Gazebo topic timeout")
        except Exception as e:
            self.get_logger().error(f"Error polling camera: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GzCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

