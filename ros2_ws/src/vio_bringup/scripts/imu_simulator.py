#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time
import numpy as np

class IMUSimulator(Node):
    """Simulates IMU measurements (accelerometer + gyroscope) for VIO system"""

    def __init__(self):
        super().__init__('imu_simulator')

        self.publisher = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.timer = self.create_timer(1.0/100.0, self.publish_imu_data)  # 100 Hz IMU

        # Motion state (must match camera simulator exactly)
        self.start_time = time.time()
        self.position = np.array([0.0, 0.0, -2.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.yaw = 0.0
        self.yaw_rate = 0.0

        # Previous state for derivative calculations
        self.prev_position = self.position.copy()
        self.prev_yaw = self.yaw
        self.prev_time = self.start_time

        # IMU biases and noise (realistic values)
        self.accel_bias = np.array([0.0, 0.0, 9.81])  # Include gravity
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.accel_noise_std = 0.01  # m/s²
        self.gyro_noise_std = 0.001  # rad/s

        self.get_logger().info('IMU simulator started: 100Hz IMU data with realistic noise')

    def update_motion(self):
        """Update drone position/orientation (must match camera simulator exactly)"""
        elapsed = time.time() - self.start_time

        # Store previous state for derivative calculations
        self.prev_position = self.position.copy()
        self.prev_yaw = self.yaw
        dt = time.time() - self.prev_time
        self.prev_time = time.time()

        # Simulate drone motion (same as camera simulator)
        if elapsed < 5.0:  # Takeoff
            self.position[2] = -2.0 + elapsed * 0.2  # Climb
            self.velocity = np.array([0.0, 0.0, 0.2])  # Constant climb velocity
            self.yaw = 0.0
            self.yaw_rate = 0.0
        elif elapsed < 15.0:  # Figure-8 cruise
            angle = (elapsed - 5.0) * 0.5
            self.position[0] = 2.0 * math.sin(angle)
            self.position[1] = 1.0 * math.sin(2 * angle)
            self.position[2] = -2.0

            # Calculate velocity (derivative of position)
            self.velocity[0] = 2.0 * 0.5 * math.cos(angle)  # d/dt of 2*sin(0.5*t)
            self.velocity[1] = 1.0 * 2 * 0.5 * math.cos(2 * angle)  # d/dt of sin(2*0.5*t)
            self.velocity[2] = 0.0

            self.yaw = angle  # Face direction of motion
            self.yaw_rate = 0.5  # Constant yaw rate
        else:  # Landing
            self.position[2] = -2.0 - (elapsed - 15.0) * 0.1
            self.velocity = np.array([0.0, 0.0, -0.1])  # Constant descent velocity
            self.yaw = math.pi  # Face backward during landing
            self.yaw_rate = 0.0  # Stop yaw rotation

    def calculate_imu_measurements(self):
        """Calculate accelerometer and gyroscope measurements from motion"""
        # Calculate acceleration (derivative of velocity + gravity)
        # In NED frame: acceleration = d²position/dt² + gravity
        accel_ned = np.array([0.0, 0.0, 0.0])  # We'll use finite differences

        # For more realistic simulation, add some acceleration changes
        # during maneuvers
        elapsed = time.time() - self.start_time
        if 4.5 < elapsed < 5.5:  # Transition to cruise
            accel_ned = np.array([0.0, 0.0, -0.5])  # Deceleration in climb
        elif 14.5 < elapsed < 15.5:  # Transition to landing
            accel_ned = np.array([0.0, 0.0, 0.5])  # Acceleration downward

        # Transform to body frame (rotate by yaw)
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        # NED to body frame rotation (only yaw rotation)
        accel_body = np.array([
            accel_ned[0] * cos_yaw + accel_ned[1] * sin_yaw,
            -accel_ned[0] * sin_yaw + accel_ned[1] * cos_yaw,
            accel_ned[2]
        ])

        # Add gravity (in body frame, gravity is [0, 0, -9.81] when level)
        accel_body[2] -= 9.81

        # Gyroscope measurements (angular rates in body frame)
        gyro_body = np.array([
            0.0,  # roll rate
            0.0,  # pitch rate
            self.yaw_rate  # yaw rate
        ])

        return accel_body, gyro_body

    def publish_imu_data(self):
        """Generate and publish IMU data"""
        # Update motion state
        self.update_motion()

        # Calculate IMU measurements
        accel_body, gyro_body = self.calculate_imu_measurements()

        # Add biases and noise
        accel_measured = accel_body + self.accel_bias + np.random.normal(0, self.accel_noise_std, 3)
        gyro_measured = gyro_body + self.gyro_bias + np.random.normal(0, self.gyro_noise_std, 3)

        # Create IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Linear acceleration (m/s²)
        imu_msg.linear_acceleration.x = accel_measured[0]
        imu_msg.linear_acceleration.y = accel_measured[1]
        imu_msg.linear_acceleration.z = accel_measured[2]

        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = gyro_measured[0]
        imu_msg.angular_velocity.y = gyro_measured[1]
        imu_msg.angular_velocity.z = gyro_measured[2]

        # Orientation (not measured by IMU - leave as identity)
        imu_msg.orientation.w = 1.0
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0

        # Covariances (realistic IMU covariances)
        imu_msg.linear_acceleration_covariance = [
            self.accel_noise_std**2, 0, 0,
            0, self.accel_noise_std**2, 0,
            0, 0, self.accel_noise_std**2
        ]
        imu_msg.angular_velocity_covariance = [
            self.gyro_noise_std**2, 0, 0,
            0, self.gyro_noise_std**2, 0,
            0, 0, self.gyro_noise_std**2
        ]
        imu_msg.orientation_covariance = [-1] * 9  # Unknown orientation

        # Publish
        self.publisher.publish(imu_msg)
        self.get_logger().debug('.3f')


def main(args=None):
    rclpy.init(args=args)
    node = IMUSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

