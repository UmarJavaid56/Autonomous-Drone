#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
import cv2
import numpy as np
import math
import time
import os

class GazeboStyleCameraSimulator(Node):
    """Simulates camera images similar to what Gazebo would provide"""

    def __init__(self):
        super().__init__('gazebo_camera_simulator')

        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.depth_publisher = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.imu_publisher = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.timer = self.create_timer(1.0/30.0, self.publish_camera_image)  # 30 Hz like Gazebo
        self.imu_timer = self.create_timer(1.0/100.0, self.publish_imu_data)  # 100 Hz IMU

        # Camera properties (matching Gazebo x500_mono_cam)
        self.width = 1280
        self.height = 960
        self.fov_degrees = 100  # ~100° horizontal FOV

        self.timer = self.create_timer(1.0/30.0, self.publish_camera_image)

        # Motion state (simulating drone movement)
        self.start_time = time.time()
        self.position = np.array([0.0, 0.0, -2.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.yaw = 0.0
        self.yaw_rate = 0.0

        # IMU state tracking
        self.last_position = self.position.copy()
        self.last_yaw = self.yaw
        self.last_time = self.start_time

        # Timestamp tracking for PX4 compatibility (monotonically increasing)
        self.timestamp_base = int(time.time() * 1e6)  # microseconds since epoch
        self.imu_sample_count = 0
        self.camera_sample_count = 0
        self.imu_sample_interval = int(1e6 / 100)  # 100Hz = 10ms = 10000 microseconds
        self.camera_sample_interval = int(1e6 / 30)  # 30Hz ≈ 33.3ms = 33333 microseconds

        # IMU noise and bias (realistic values)
        self.accel_noise_std = 0.01  # m/s²
        self.gyro_noise_std = 0.001  # rad/s
        self.accel_bias = np.array([0.0, 0.0, 9.81])  # Include gravity
        self.gyro_bias = np.array([0.0, 0.0, 0.0])

        self.get_logger().info(f'Gazebo-style camera simulator started: {self.width}x{self.height} RGB+Depth @ 30Hz + IMU @ 100Hz')

    def update_motion(self):
        """Update camera position/orientation to simulate drone movement"""
        current_time = time.time()
        elapsed = current_time - self.start_time
        dt = current_time - self.last_time

        # Store previous state for derivative calculations
        self.last_position = self.position.copy()
        self.last_yaw = self.yaw

        # Simulate drone motion similar to our SLAM simulator
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

        self.last_time = current_time

    def generate_camera_image(self):
        """Generate a camera image that looks like Gazebo output"""

        # Create base image
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Add ground plane (gradient from sky to ground)
        for y in range(self.height):
            if y < self.height // 2:  # Sky
                color = [200, 220, 255]  # Light blue sky
            else:  # Ground
                ground_factor = (y - self.height // 2) / (self.height // 2)
                color = [int(100 + ground_factor * 50),  # Brown/green ground
                        int(80 + ground_factor * 60),
                        int(60 + ground_factor * 40)]
            cv2.line(img, (0, y), (self.width, y), color)

        # Add horizon line
        horizon_y = self.height // 2 + int(self.position[2] * 100)  # Move with altitude
        cv2.line(img, (0, horizon_y), (self.width, horizon_y), (255, 255, 255), 2)

        # Add motion blur effect (realistic for camera)
        if abs(self.position[0]) > 0.5 or abs(self.position[1]) > 0.5:  # If moving fast
            img = cv2.GaussianBlur(img, (5, 5), 0)

        # Add some "features" that SLAM can track (corners, edges)
        # Simulate buildings/obstacles
        for i in range(5):
            x = int(self.width//2 + (i-2) * 200 + self.position[0] * 50)
            if 0 <= x < self.width:
                # Building-like structures
                building_height = int(100 + i * 30)
                cv2.rectangle(img, (x, horizon_y - building_height),
                            (x + 40, horizon_y), (100, 100, 120), -1)
                # Add windows/corners for SLAM tracking
                cv2.rectangle(img, (x+5, horizon_y - building_height + 10),
                            (x+15, horizon_y - building_height + 30), (200, 200, 255), -1)

        # Add text overlay (like Gazebo camera info)
        status = "TRACKING" if time.time() % 10 < 9 else "LOST"  # Occasional tracking loss
        cv2.putText(img, f"Gazebo Camera | Pos: {self.position[0]:.1f}, {self.position[1]:.1f}, {self.position[2]:.1f}",
                   (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(img, f"Yaw: {self.yaw:.1f} rad | Status: {status}",
                   (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Add realistic camera effects (simplified - no distortion for now)
        # Lens distortion would be added here in a more complete implementation

        # Add some noise (camera sensor noise)
        noise = np.random.normal(0, 2, img.shape).astype(np.uint8)
        img = cv2.add(img, noise)

        return img

    def generate_depth_image(self):
        """Generate a depth image (distance in meters) corresponding to the RGB scene"""
        # Create depth map (float32, meters) - optimized for performance
        depth_map = np.full((self.height, self.width), abs(self.position[2]), dtype=np.float32)

        # Add buildings as rectangular depth variations (very fast)
        building_distance = max(1.0, abs(self.position[2]) - 2.0)

        # Create building rectangles efficiently
        for i in range(5):
            building_x = int(self.width//2 + (i-2) * 200 + self.position[0] * 50)
            building_left = max(0, building_x - 40)
            building_right = min(self.width, building_x + 40)

            if building_left < building_right:
                # Set building depth (bottom half of image)
                depth_map[self.height//2:, building_left:building_right] = building_distance

        # Add minimal noise (much faster than full noise array)
        # Only add noise to a few random pixels for realism
        noise_pixels = np.random.choice(depth_map.size, size=int(depth_map.size * 0.01), replace=False)
        noise_values = np.random.normal(0, 0.05, len(noise_pixels)).astype(np.float32)
        depth_map.flat[noise_pixels] += noise_values

        # Ensure minimum depth
        np.maximum(depth_map, 0.1, out=depth_map)

        return depth_map

    def publish_camera_image(self):
        """Generate and publish camera image"""
        # Update camera position/orientation
        self.update_motion()

        # Generate realistic camera image
        cv_image = self.generate_camera_image()

        # Convert to ROS Image message with synchronized timestamp
        ros_image = Image()
        current_timestamp = self.timestamp_base + (self.camera_sample_count * self.camera_sample_interval)
        ros_image.header.stamp.sec = current_timestamp // 1000000
        ros_image.header.stamp.nanosec = (current_timestamp % 1000000) * 1000
        ros_image.header.frame_id = 'camera_link'
        self.camera_sample_count += 1
        ros_image.height = self.height
        ros_image.width = self.width
        ros_image.encoding = 'bgr8'
        ros_image.is_bigendian = False
        ros_image.step = self.width * 3
        ros_image.data = cv_image.tobytes()

        # Publish RGB image
        self.publisher.publish(ros_image)
        self.get_logger().info(f'Published RGB camera image: {self.width}x{self.height}')

        # Generate and publish depth image
        depth_image = self.generate_depth_image()

        # Convert to ROS Image message
        ros_depth = Image()
        ros_depth.header.stamp.sec = current_timestamp // 1000000
        ros_depth.header.stamp.nanosec = (current_timestamp % 1000000) * 1000
        ros_depth.header.frame_id = 'camera_link'
        ros_depth.height = self.height
        ros_depth.width = self.width
        ros_depth.encoding = '32FC1'  # 32-bit float, 1 channel
        ros_depth.is_bigendian = False
        ros_depth.step = self.width * 4  # 4 bytes per float
        ros_depth.data = depth_image.tobytes()

        # Publish depth image
        self.depth_publisher.publish(ros_depth)
        self.get_logger().info(f'Published depth image: {self.width}x{self.height}')

    def publish_imu_data(self):
        """Generate and publish IMU data synchronized with camera motion"""
        try:
            self.get_logger().info("IMU timer fired")  # Debug message
            # Update motion state (ensure it's current)
            self.update_motion()

            # Calculate acceleration from velocity changes
            # For realistic IMU, we differentiate velocity to get acceleration
            accel_body = np.array([0.0, 0.0, 0.0])

            # Add some realistic acceleration changes during maneuvers
            elapsed = time.time() - self.start_time
            if 4.5 < elapsed < 5.5:  # Transition to cruise
                accel_body = np.array([0.0, 0.0, -0.5])  # Deceleration in climb
            elif 14.5 < elapsed < 15.5:  # Transition to landing
                accel_body = np.array([0.0, 0.0, 0.5])  # Acceleration downward

            # Transform acceleration to body frame (simple yaw rotation)
            cos_yaw = math.cos(self.yaw)
            sin_yaw = math.sin(self.yaw)
            accel_world = np.array([
                accel_body[0] * cos_yaw - accel_body[1] * sin_yaw,
                accel_body[0] * sin_yaw + accel_body[1] * cos_yaw,
                accel_body[2]
            ])

            # Add gravity (in body frame, assuming level flight)
            accel_world[2] -= 9.81

            # Gyroscope measurements (angular rates in body frame)
            gyro_body = np.array([0.0, 0.0, self.yaw_rate])

            # Add noise and bias
            accel_measured = accel_world + self.accel_bias + np.random.normal(0, self.accel_noise_std, 3)
            gyro_measured = gyro_body + self.gyro_bias + np.random.normal(0, self.gyro_noise_std, 3)

            # Create IMU message with monotonically increasing timestamp for PX4
            imu_msg = Imu()
            # Use monotonically increasing timestamp based on sample count
            current_timestamp = self.timestamp_base + (self.imu_sample_count * self.imu_sample_interval)
            imu_msg.header.stamp.sec = current_timestamp // 1000000
            imu_msg.header.stamp.nanosec = (current_timestamp % 1000000) * 1000
            imu_msg.header.frame_id = 'imu_link'
            self.imu_sample_count += 1

            # Linear acceleration
            imu_msg.linear_acceleration.x = accel_measured[0]
            imu_msg.linear_acceleration.y = accel_measured[1]
            imu_msg.linear_acceleration.z = accel_measured[2]

            # Angular velocity
            imu_msg.angular_velocity.x = gyro_measured[0]
            imu_msg.angular_velocity.y = gyro_measured[1]
            imu_msg.angular_velocity.z = gyro_measured[2]

            # Orientation (identity - unknown for IMU)
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
            self.get_logger().debug('.3f')
        except Exception as e:
            self.get_logger().error(f"Error publishing IMU data: {e}")


def main(args=None):
    # Set DDS transport configuration BEFORE rclpy.init()
    if 'RMW_IMPLEMENTATION' not in os.environ:
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    if 'FASTRTPS_DEFAULT_PROFILES_FILE' not in os.environ:
        dds_config = os.path.expanduser('~/.ros/fastdds.xml')
        if os.path.exists(dds_config):
            os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = dds_config
    if 'RMW_FASTRTPS_USE_QOS_FROM_XML' not in os.environ:
        os.environ['RMW_FASTRTPS_USE_QOS_FROM_XML'] = '1'
    if 'RMW_FASTRTPS_USE_SHARED_MEMORY' not in os.environ:
        os.environ['RMW_FASTRTPS_USE_SHARED_MEMORY'] = '0'

    try:
        rclpy.init(args=args)
        node = GazeboStyleCameraSimulator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Exception in camera simulator: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
