#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
import time
import random
import numpy as np

class RealisticSLAMSimulator(Node):
    def __init__(self):
        super().__init__('realistic_slam_simulator')

        # Publisher for pose data (simulating ORB-SLAM3 output)
        self.pose_publisher = self.create_publisher(PoseStamped, '/orbslam3/pose', 10)

        # Subscriber to get drone's actual position from PX4 (if available)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Timer to publish poses at 30 Hz
        self.timer = self.create_timer(1.0/30.0, self.publish_pose)

        # SLAM simulation state
        self.start_time = time.time()
        self.true_position = np.array([0.0, 0.0, -2.0])  # Start at origin, 2m up
        self.slam_position = np.array([0.0, 0.0, -2.0])   # SLAM estimate (starts same)
        self.velocity = np.array([0.0, 0.0, 0.0])

        # SLAM characteristics
        self.scale_drift = 0.001  # Scale factor drift
        self.position_noise = 0.02  # Position noise (2cm)
        self.tracking_failure_prob = 0.001  # 0.1% chance of tracking failure per frame
        self.is_tracking = True
        self.last_tracking_time = time.time()

        # Motion pattern
        self.motion_phase = 0
        self.max_speed = 2.0  # m/s

        self.get_logger().info('Realistic SLAM Simulator started - motion-coupled pose generation')

    def odom_callback(self, msg):
        """Receive drone's actual position from PX4"""
        # Update true position and velocity from PX4 odometry
        self.true_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        # Extract velocity if available
        if hasattr(msg.twist.twist, 'linear'):
            self.velocity = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            ])

    def publish_pose(self):
        """Generate realistic SLAM pose based on simulated motion and SLAM characteristics"""
        current_time = time.time()
        dt = 1.0/30.0  # 30 Hz

        # Generate motion if no odometry data (fallback to scripted motion)
        if not hasattr(self, 'last_update_time'):
            self.last_update_time = current_time
            self.generate_motion(dt)

        # Update SLAM simulation
        self.update_slam_simulation(dt)

        # Create pose message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'

        # Set SLAM-estimated position (with errors)
        pose_msg.pose.position.x = self.slam_position[0]
        pose_msg.pose.position.y = self.slam_position[1]
        pose_msg.pose.position.z = self.slam_position[2]

        # Set orientation (simplified - facing direction of motion)
        yaw = math.atan2(self.velocity[1], self.velocity[0]) if np.linalg.norm(self.velocity) > 0.1 else 0.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(yaw/2)
        pose_msg.pose.orientation.w = math.cos(yaw/2)

        # Publish pose
        self.pose_publisher.publish(pose_msg)

        # Log status
        tracking_status = "TRACKING" if self.is_tracking else "LOST"
        self.get_logger().info('.3f')

    def generate_motion(self, dt):
        """Generate realistic drone motion pattern"""
        elapsed = time.time() - self.start_time

        # Different motion phases: takeoff, cruise, landing
        if elapsed < 5.0:  # Takeoff phase
            self.velocity = np.array([0.0, 0.0, 1.0])  # Climb
        elif elapsed < 15.0:  # Cruise phase - figure 8 pattern
            angle = (elapsed - 5.0) * 0.5
            vx = 2.0 * math.sin(angle)
            vy = 1.0 * math.sin(2 * angle)  # Figure 8
            vz = 0.0
            self.velocity = np.array([vx, vy, vz])
        else:  # Landing phase
            self.velocity = np.array([0.0, 0.0, -0.5])  # Descend

        # Update true position
        self.true_position += self.velocity * dt

    def update_slam_simulation(self, dt):
        """Simulate realistic SLAM behavior"""
        # Simulate tracking failures
        if random.random() < self.tracking_failure_prob:
            self.is_tracking = False
            self.last_tracking_time = time.time()
            self.get_logger().warn("SLAM tracking lost!")

        # Recovery from tracking failure (takes 1-3 seconds)
        if not self.is_tracking and (time.time() - self.last_tracking_time) > random.uniform(1.0, 3.0):
            self.is_tracking = True
            self.get_logger().info("SLAM tracking recovered")

        if self.is_tracking:
            # Normal SLAM operation with realistic errors
            # Motion-based pose update
            motion_estimate = self.velocity * dt

            # Add SLAM errors
            motion_noise = np.random.normal(0, self.position_noise, 3)
            scale_error = 1.0 + random.gauss(0, self.scale_drift)

            motion_estimate = motion_estimate * scale_error + motion_noise

            # Update SLAM position
            self.slam_position += motion_estimate

            # Add bias drift over time
            drift = np.random.normal(0, 0.001, 3)  # Slow drift
            self.slam_position += drift
        else:
            # When tracking is lost, SLAM position stays constant (dead reckoning would take over)
            pass

def main(args=None):
    rclpy.init(args=args)
    node = RealisticSLAMSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
