#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
import numpy as np

class GzPhysicsImuBridge(Node):
    """Generate IMU data from Gazebo odometry/physics data"""

    def __init__(self):
        super().__init__('gz_physics_imu_bridge')

        # ROS publisher
        self.imu_publisher = self.create_publisher(Imu, '/imu/data_raw', 10)

        # Subscribe to Gazebo odometry (which contains velocity information)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/x500_0/odometry_with_covariance',
            self.odom_callback,
            10
        )

        # IMU properties
        self.last_timestamp = None
        self.last_linear_vel = np.array([0.0, 0.0, 0.0])
        self.last_angular_vel = np.array([0.0, 0.0, 0.0])
        self.last_accel = np.array([0.0, 0.0, 0.0])

        # Noise characteristics (realistic IMU noise)
        self.accel_noise_std = 0.01  # m/s²
        self.gyro_noise_std = 0.001  # rad/s
        self.accel_bias = np.array([0.0, 0.0, 9.81])  # Include gravity
        self.gyro_bias = np.array([0.0, 0.0, 0.0])

        self.get_logger().info('Gazebo physics-based IMU bridge started - using odometry for realistic IMU data')

    def odom_callback(self, odom_msg):
        """Process odometry data to generate IMU measurements"""
        try:
            current_time = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9

            # Extract velocities from odometry
            linear_vel = np.array([
                odom_msg.twist.twist.linear.x,
                odom_msg.twist.twist.linear.y,
                odom_msg.twist.twist.linear.z
            ])

            angular_vel = np.array([
                odom_msg.twist.twist.angular.x,
                odom_msg.twist.twist.angular.y,
                odom_msg.twist.twist.angular.z
            ])

            # Calculate accelerations (finite difference)
            if self.last_timestamp is not None:
                dt = current_time - self.last_timestamp
                if dt > 0:
                    # Calculate acceleration from velocity change
                    accel_body = (linear_vel - self.last_linear_vel) / dt

                    # Add gravity (in body frame, assuming NED to body conversion)
                    # For simplicity, assume the drone is mostly level
                    accel_body[2] -= 9.81

                    # Add some smoothing to reduce noise from differentiation
                    alpha = 0.1  # Low-pass filter coefficient
                    self.last_accel = alpha * accel_body + (1 - alpha) * self.last_accel
                    accel_body = self.last_accel

                    # Add noise and bias
                    accel_measured = accel_body + self.accel_bias + np.random.normal(0, self.accel_noise_std, 3)
                    gyro_measured = angular_vel + self.gyro_bias + np.random.normal(0, self.gyro_noise_std, 3)

                    # Create IMU message
                    imu_msg = Imu()
                    imu_msg.header.stamp = odom_msg.header.stamp
                    imu_msg.header.frame_id = 'imu_link'

                    # Linear acceleration
                    imu_msg.linear_acceleration.x = accel_measured[0]
                    imu_msg.linear_acceleration.y = accel_measured[1]
                    imu_msg.linear_acceleration.z = accel_measured[2]

                    # Angular velocity
                    imu_msg.angular_velocity.x = gyro_measured[0]
                    imu_msg.angular_velocity.y = gyro_measured[1]
                    imu_msg.angular_velocity.z = gyro_measured[2]

                    # Orientation (unknown for IMU)
                    imu_msg.orientation.w = 1.0
                    imu_msg.orientation.x = 0.0
                    imu_msg.orientation.y = 0.0
                    imu_msg.orientation.z = 0.0

                    # Covariances
                    accel_cov = self.accel_noise_std**2
                    gyro_cov = self.gyro_noise_std**2
                    imu_msg.linear_acceleration_covariance = [
                        accel_cov, 0, 0,
                        0, accel_cov, 0,
                        0, 0, accel_cov
                    ]
                    imu_msg.angular_velocity_covariance = [
                        gyro_cov, 0, 0,
                        0, gyro_cov, 0,
                        0, 0, gyro_cov
                    ]
                    imu_msg.orientation_covariance = [-1] * 9

                    # Publish
                    self.imu_publisher.publish(imu_msg)

                    self.get_logger().debug('.3f'
                                           '.3f')

            # Update previous values
            self.last_timestamp = current_time
            self.last_linear_vel = linear_vel.copy()
            self.last_angular_vel = angular_vel.copy()

        except Exception as e:
            self.get_logger().error(f"Error processing odometry: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GzPhysicsImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

