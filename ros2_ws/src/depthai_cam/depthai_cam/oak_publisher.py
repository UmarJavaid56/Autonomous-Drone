#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import depthai as dai
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class OakPublisher(Node):
    def __init__(self):
        super().__init__('oak_publisher')

        # Parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('topic', 'camera/image_raw')

        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        self.topic_name = self.get_parameter('topic').get_parameter_value().string_value

        self.get_logger().info(
            f"Starting OAK publisher at {self.width}x{self.height} @ {self.fps}Hz on '{self.topic_name}'"
        )

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, self.topic_name, 10)

        # ---- DepthAI pipeline (matching your working demo) ----
        self.pipeline = dai.Pipeline()

        camRgb = self.pipeline.create(dai.node.ColorCamera)
        camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setPreviewSize(self.width, self.height)
        camRgb.setFps(self.fps)

        # Create output queue from the preview stream
        self.videoQueue = camRgb.preview.createOutputQueue(maxSize=4, blocking=False)

        # Start device
        self.device = self.pipeline.start()

        # Timer to poll frames
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Get latest frame (non-blocking)
        in_frame = self.videoQueue.tryGet()
        if in_frame is None:
            return

        frame = in_frame.getCvFrame()
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "oak_rgb_optical_frame"

        self.publisher_.publish(msg)


def main(args=None):
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

