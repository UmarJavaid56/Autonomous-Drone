#!/usr/bin/env python3
"""
SLAM Status Checker - Monitors VIO system health and performance
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, Imu
import time
import numpy as np

class SLAMStatusChecker(Node):
    def __init__(self):
        super().__init__('slam_status_checker')

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/orbslam3/pose_processed', self.pose_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)

        # Status tracking
        self.last_pose_time = 0
        self.last_camera_time = 0
        self.last_depth_time = 0
        self.last_imu_time = 0
        self.pose_count = 0
        self.camera_count = 0
        self.depth_count = 0
        self.imu_count = 0

        # Performance tracking
        self.pose_positions = []
        self.start_time = time.time()

        # Timer for periodic status reports
        self.timer = self.create_timer(5.0, self.print_status)

        self.get_logger().info('SLAM Status Checker started - monitoring VIO system health')

    def pose_callback(self, msg):
        self.last_pose_time = time.time()
        self.pose_count += 1

        # Track position for drift analysis
        pos = msg.pose.position
        self.pose_positions.append([pos.x, pos.y, pos.z])

        # Keep only last 100 positions for analysis
        if len(self.pose_positions) > 100:
            self.pose_positions.pop(0)

    def camera_callback(self, msg):
        self.last_camera_time = time.time()
        self.camera_count += 1

    def depth_callback(self, msg):
        self.last_depth_time = time.time()
        self.depth_count += 1

    def imu_callback(self, msg):
        self.last_imu_time = time.time()
        self.imu_count += 1

    def print_status(self):
        current_time = time.time()
        uptime = current_time - self.start_time

        self.get_logger().info('='*60)
        self.get_logger().info('SLAM SYSTEM STATUS REPORT')
        self.get_logger().info('='*60)

        # Data rates
        self.get_logger().info(f'Uptime: {uptime:.1f}s')
        self.get_logger().info(f'Pose Rate: {self.pose_count/uptime:.1f} Hz ({self.pose_count} total)')
        self.get_logger().info(f'Camera Rate: {self.camera_count/uptime:.1f} Hz ({self.camera_count} total)')
        self.get_logger().info(f'Depth Rate: {self.depth_count/uptime:.1f} Hz ({self.depth_count} total)')
        self.get_logger().info(f'IMU Rate: {self.imu_count/uptime:.1f} Hz ({self.imu_count} total)')

        # Freshness checks
        pose_age = current_time - self.last_pose_time if self.last_pose_time > 0 else float('inf')
        camera_age = current_time - self.last_camera_time if self.last_camera_time > 0 else float('inf')
        depth_age = current_time - self.last_depth_time if self.last_depth_time > 0 else float('inf')
        imu_age = current_time - self.last_imu_time if self.last_imu_time > 0 else float('inf')

        self.get_logger().info('Data Freshness (age in seconds):')
        self.get_logger().info(f'  Pose: {pose_age:.3f}s')
        self.get_logger().info(f'  Camera: {camera_age:.3f}s')
        self.get_logger().info(f'  Depth: {depth_age:.3f}s')
        self.get_logger().info(f'  IMU: {imu_age:.3f}s')

        # Position analysis
        if len(self.pose_positions) > 10:
            positions = np.array(self.pose_positions)
            pos_std = np.std(positions, axis=0)
            pos_range = np.ptp(positions, axis=0)

            self.get_logger().info('Position Statistics (last 100 samples):')
            self.get_logger().info(f'  X range: {pos_range[0]:.3f}m, std: {pos_std[0]:.3f}m')
            self.get_logger().info(f'  Y range: {pos_range[1]:.3f}m, std: {pos_std[1]:.3f}m')
            self.get_logger().info(f'  Z range: {pos_range[2]:.3f}m, std: {pos_std[2]:.3f}m')

        # Health assessment
        healthy = True
        issues = []

        if pose_age > 1.0:
            healthy = False
            issues.append("No recent pose data (>1s old)")
        if camera_age > 1.0:
            healthy = False
            issues.append("No recent camera data (>1s old)")
        if depth_age > 1.0:
            healthy = False
            issues.append("No recent depth data (>1s old)")
        if imu_age > 1.0:
            healthy = False
            issues.append("No recent IMU data (>1s old)")

        if self.pose_count == 0:
            healthy = False
            issues.append("No pose messages received")

        if healthy:
            self.get_logger().info('✅ SYSTEM HEALTH: GOOD - All data streams active')
        else:
            self.get_logger().error('❌ SYSTEM HEALTH: ISSUES DETECTED')
            for issue in issues:
                self.get_logger().error(f'  - {issue}')

        self.get_logger().info('='*60)

def main():
    rclpy.init()
    node = SLAMStatusChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
