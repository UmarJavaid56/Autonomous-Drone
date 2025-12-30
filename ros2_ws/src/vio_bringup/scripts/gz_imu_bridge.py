#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import subprocess
import json
import time
import re

class GzImuBridge(Node):
    """Bridge Gazebo IMU sensor data to ROS2 using command-line polling"""

    def __init__(self):
        super().__init__('gz_imu_bridge')

        # ROS publisher
        self.imu_publisher = self.create_publisher(Imu, '/imu/data_raw', 10)

        # Gazebo IMU topic
        self.gz_topic = '/world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu'

        # Timer to poll Gazebo topic (100 Hz IMU)
        self.timer = self.create_timer(0.01, self.poll_imu)  # 100 Hz

        self.get_logger().info('Gazebo IMU bridge started - polling IMU data at 100Hz')

        # Previous IMU data for interpolation
        self.last_imu_data = None

    def poll_imu(self):
        """Poll Gazebo IMU topic and publish to ROS"""
        try:
            # Use gz topic echo to get IMU data (get 1 message)
            cmd = ['gz', 'topic', '-e', '-n', '1', '-t', '2', self.gz_topic]  # 2 second timeout
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=3)

            if result.returncode == 0 and result.stdout.strip():
                # Parse the gz message output
                # Gz topic echo outputs JSON-like data, but we need to extract IMU values
                imu_data = self.parse_gz_imu_output(result.stdout)

                if imu_data:
                    # Create ROS IMU message
                    ros_imu = Imu()
                    ros_imu.header.stamp = self.get_clock().now().to_msg()
                    ros_imu.header.frame_id = 'imu_link'

                    # Linear acceleration
                    ros_imu.linear_acceleration.x = imu_data['accel_x']
                    ros_imu.linear_acceleration.y = imu_data['accel_y']
                    ros_imu.linear_acceleration.z = imu_data['accel_z']

                    # Angular velocity
                    ros_imu.angular_velocity.x = imu_data['gyro_x']
                    ros_imu.angular_velocity.y = imu_data['gyro_y']
                    ros_imu.angular_velocity.z = imu_data['gyro_z']

                    # Orientation (identity if not available)
                    ros_imu.orientation.w = 1.0
                    ros_imu.orientation.x = 0.0
                    ros_imu.orientation.y = 0.0
                    ros_imu.orientation.z = 0.0

                    # Covariances
                    accel_cov = 0.01  # m/s²
                    gyro_cov = 0.001   # rad/s
                    ros_imu.linear_acceleration_covariance = [
                        accel_cov, 0, 0,
                        0, accel_cov, 0,
                        0, 0, accel_cov
                    ]
                    ros_imu.angular_velocity_covariance = [
                        gyro_cov, 0, 0,
                        0, gyro_cov, 0,
                        0, 0, gyro_cov
                    ]
                    ros_imu.orientation_covariance = [-1] * 9

                    # Publish
                    self.imu_publisher.publish(ros_imu)
                    self.get_logger().debug('.3f')

                    self.last_imu_data = imu_data
                else:
                    # If parsing failed, use last known good data or skip
                    if self.last_imu_data:
                        self.get_logger().warn("IMU parsing failed, using last known data")
                        # Could publish last known data here if needed

        except subprocess.TimeoutExpired:
            self.get_logger().warn("Gazebo IMU topic timeout")
        except Exception as e:
            self.get_logger().error(f"Error polling IMU: {e}")

    def parse_gz_imu_output(self, output):
        """Parse gz topic echo output to extract IMU data"""
        try:
            # The gz topic output is complex. Let's try to extract key values using regex
            # Look for patterns like: linear_acceleration { x: 0.1 y: 0.2 z: 9.8 }
            # and angular_velocity { x: 0.01 y: 0.02 z: 0.03 }

            accel_match = re.search(r'linear_acceleration\s*\{\s*x:\s*([-\d.]+)\s+y:\s*([-\d.]+)\s+z:\s*([-\d.]+)', output)
            gyro_match = re.search(r'angular_velocity\s*\{\s*x:\s*([-\d.]+)\s+y:\s*([-\d.]+)\s+z:\s*([-\d.]+)', output)

            if accel_match and gyro_match:
                return {
                    'accel_x': float(accel_match.group(1)),
                    'accel_y': float(accel_match.group(2)),
                    'accel_z': float(accel_match.group(3)),
                    'gyro_x': float(gyro_match.group(1)),
                    'gyro_y': float(gyro_match.group(2)),
                    'gyro_z': float(gyro_match.group(3))
                }
            else:
                # Try alternative parsing if the regex doesn't match
                self.get_logger().debug(f"IMU output parsing failed. Output: {output[:200]}...")
                return None

        except Exception as e:
            self.get_logger().error(f"Error parsing IMU output: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = GzImuBridge()

    try:
        # Use a multi-threaded executor to handle both ROS and Gazebo callbacks
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
