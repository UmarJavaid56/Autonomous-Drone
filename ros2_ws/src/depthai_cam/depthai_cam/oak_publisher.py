#!/usr/bin/env python3
"""ROS 2 node for publishing OAK-D Lite camera streams.

This node interfaces with an OAK-D Lite camera using the DepthAI SDK
and publishes RGBD image streams to ROS 2 topics.

For simulation, the Gazebo model provides camera streams directly via
ros_gz_bridge.
"""
import rclpy
from rclpy.node import Node

import depthai as dai
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class OakPublisher(Node):
    """ROS 2 node for OAK-D Lite camera RGBD stream publishing.
    
    This node creates a DepthAI pipeline to capture RGBD frames from the
    OAK-D Lite camera and publishes them as ROS Image messages.
    """

    def __init__(self):
        super().__init__('oak_publisher')

        # Declare and get parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('topic', 'camera/image_raw')

        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        self.topic_name = self.get_parameter('topic').get_parameter_value().string_value

        self.get_logger().info(
            f'Starting OAK publisher at {self.width}x{self.height} @ {self.fps}Hz on \'{self.topic_name}\''
        )

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, self.topic_name, 10)

        # Initialize DepthAI pipeline
        self.pipeline = dai.Pipeline()

        # Configure RGB camera
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setPreviewSize(self.width, self.height)
        cam_rgb.setFps(self.fps)

        # Create output queue from the preview stream
        self.video_queue = cam_rgb.preview.createOutputQueue(maxSize=4, blocking=False)

        # Start device
        self.device = self.pipeline.start()

        # Timer to poll frames at specified rate
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """Poll for new frames and publish to ROS topic."""
        # Get latest frame (non-blocking)
        in_frame = self.video_queue.tryGet()
        if in_frame is None:
            return

        # Convert to ROS Image message
        frame = in_frame.getCvFrame()
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'oak_rgb_optical_frame'

        self.publisher_.publish(msg)


def main(args=None):
    """Entry point for the OAK publisher node."""
    rclpy.init(args=args)
    node = OakPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

