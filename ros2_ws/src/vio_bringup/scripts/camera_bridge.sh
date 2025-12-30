#!/bin/bash

# Simple camera bridge using gz topic echo
# This is a basic implementation - in production you'd want proper message parsing

echo "Starting Gazebo camera to ROS bridge..."

# ROS 2 setup
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=1

# Create a simple ROS publisher script
cat > /tmp/camera_publisher.py << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time

class SimpleCameraPublisher(Node):
    def __init__(self):
        super().__init__('simple_camera_publisher')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.get_logger().info('Simple camera publisher started')

        # Publish dummy images for now (to test pipeline)
        self.timer = self.create_timer(0.1, self.publish_dummy_image)  # 10 Hz

    def publish_dummy_image(self):
        # Create a test pattern image
        img = np.zeros((480, 640, 3), dtype=np.uint8)

        # Add some test pattern
        cv2.circle(img, (320, 240), 50, (0, 255, 0), 2)
        cv2.putText(img, f"Sim Camera {time.time():.1f}",
                   (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # Convert to ROS message
        ros_img = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        ros_img.header.stamp = self.get_clock().now().to_msg()
        ros_img.header.frame_id = "camera_link"

        self.publisher.publish(ros_img)
        self.get_logger().info('Published test camera image')

rclpy.init()
node = SimpleCameraPublisher()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
EOF

chmod +x /tmp/camera_publisher.py
python3 /tmp/camera_publisher.py

