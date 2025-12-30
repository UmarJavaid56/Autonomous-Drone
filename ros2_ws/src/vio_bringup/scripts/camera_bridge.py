#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Import Gazebo message types if available
try:
    from gz.msgs import Image as GzImage
    from gz.transport import Node as GzNode
    GZ_AVAILABLE = True
except ImportError:
    GZ_AVAILABLE = False
    print("Gazebo Python bindings not available, camera bridge will not work")


class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera_bridge')

        if not GZ_AVAILABLE:
            self.get_logger().error("Gazebo Python bindings not available. Install with: pip install gz-msgs")
            return

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        # Gazebo transport node
        self.gz_node = GzNode()
        self.gz_topic = '/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image'

        # Subscribe to Gazebo camera topic
        if not self.gz_node.subscribe(self.gz_topic, self.gz_callback):
            self.get_logger().error(f"Failed to subscribe to Gazebo topic: {self.gz_topic}")
        else:
            self.get_logger().info(f"Subscribed to Gazebo topic: {self.gz_topic}")

    def gz_callback(self, gz_msg):
        """Convert Gazebo Image message to ROS2 Image message"""
        try:
            # Convert gz.msgs.Image to cv::Mat
            height = gz_msg.height()
            width = gz_msg.width()
            channels = 3 if gz_msg.pixel_format_type() == 3 else 1  # RGB or grayscale

            # Get image data
            img_data = gz_msg.data()
            img_array = np.frombuffer(img_data, dtype=np.uint8)

            # Reshape based on format
            if gz_msg.pixel_format_type() == 3:  # RGB
                img_array = img_array.reshape((height, width, 3))
                # Convert RGB to BGR for OpenCV
                img_array = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
            else:
                img_array = img_array.reshape((height, width))

            # Convert to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(img_array, encoding="bgr8")

            # Set header
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_link"

            # Publish
            self.publisher.publish(ros_image)
            self.get_logger().debug(f"Published camera image: {width}x{height}")

        except Exception as e:
            self.get_logger().error(f"Error processing camera image: {e}")


def main(args=None):
    rclpy.init(args=args)

    if not GZ_AVAILABLE:
        print("Cannot run camera bridge without Gazebo Python bindings")
        return

    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

