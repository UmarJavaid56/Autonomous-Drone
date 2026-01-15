#!/usr/bin/env python3
"""Keyboard teleop node for x500_depth drone control.

Control the drone using keyboard commands:
    W/S: Forward/Backward (linear.x)
    A/D: Left/Right strafe (linear.y)
    Q/E: Rotate CCW/CW (angular.z)
    R/F: Up/Down (linear.z)
    Space: Stop all motion
    T: Toggle controller enable/disable
    Esc: Exit

This node reads keyboard input and publishes Twist messages
for velocity control of the quadcopter in Gazebo.
"""
import sys
import select
import termios
import tty
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


# Key bindings for drone control
MOVE_BINDINGS = {
    'w': (1.0, 0.0, 0.0, 0.0),   # Forward
    's': (-1.0, 0.0, 0.0, 0.0),  # Backward
    'a': (0.0, 1.0, 0.0, 0.0),   # Strafe left
    'd': (0.0, -1.0, 0.0, 0.0),  # Strafe right
    'q': (0.0, 0.0, 0.0, 1.0),   # Rotate CCW (yaw left)
    'e': (0.0, 0.0, 0.0, -1.0),  # Rotate CW (yaw right)
    'r': (0.0, 0.0, 1.0, 0.0),   # Up
    'f': (0.0, 0.0, -1.0, 0.0),  # Down
}

# Speed adjustment keys
SPEED_BINDINGS = {
    'i': (1.1, 1.0),  # Increase linear speed
    'k': (0.9, 1.0),  # Decrease linear speed
    'o': (1.0, 1.1),  # Increase angular speed
    'l': (1.0, 0.9),  # Decrease angular speed
}

USAGE_MSG = """
╔════════════════════════════════════════════════════════════════╗
║                  X500 Depth Keyboard Teleop                    ║
╠════════════════════════════════════════════════════════════════╣
║  Movement Controls:                                             ║
║       W                                                         ║
║     A   D    - Forward/Backward/Strafe Left/Right              ║
║       S                                                         ║
║                                                                 ║
║     Q / E   - Rotate CCW / CW (Yaw)                            ║
║     R / F   - Up / Down (Altitude)                             ║
║                                                                 ║
║  Speed Controls:                                                ║
║     I / K   - Increase / Decrease linear speed                 ║
║     O / L   - Increase / Decrease angular speed                ║
║                                                                 ║
║  Other:                                                         ║
║     Space   - Stop all motion (hover)                          ║
║     T       - Toggle controller enable/disable                 ║
║     Esc     - Exit teleop                                      ║
╚════════════════════════════════════════════════════════════════╝

Current speeds: linear={linear_speed:.2f} m/s, angular={angular_speed:.2f} rad/s
Controller: {status}
"""


class KeyboardTeleopNode(Node):
    """ROS2 node for keyboard-based velocity control."""

    def __init__(self):
        super().__init__('keyboard_teleop')

        # Declare parameters
        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('cmd_vel_topic', '/x500_depth/cmd_vel')
        self.declare_parameter('enable_topic', '/x500_depth/enable')
        self.declare_parameter('publish_rate', 20.0)

        # Get parameter values
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        enable_topic = self.get_parameter('enable_topic').value
        publish_rate = self.get_parameter('publish_rate').value

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.enable_pub = self.create_publisher(Bool, enable_topic, 10)

        # Timer for continuous publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        # Current velocity state
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.angular_z = 0.0

        # Controller state
        self.enabled = False

        # Terminal settings for raw input
        self.settings: Optional[list] = None

        self.get_logger().info(f'Publishing velocity to: {cmd_vel_topic}')
        self.get_logger().info(f'Enable topic: {enable_topic}')

    def get_key(self, timeout: float = 0.1) -> str:
        """Read a single keypress from stdin with timeout."""
        if select.select([sys.stdin], [], [], timeout)[0]:
            return sys.stdin.read(1)
        return ''

    def timer_callback(self):
        """Publish current velocity state."""
        twist = Twist()
        twist.linear.x = self.linear_x * self.linear_speed
        twist.linear.y = self.linear_y * self.linear_speed
        twist.linear.z = self.linear_z * self.linear_speed
        twist.angular.z = self.angular_z * self.angular_speed
        self.cmd_vel_pub.publish(twist)

    def toggle_enable(self):
        """Toggle the velocity controller enable state."""
        self.enabled = not self.enabled
        msg = Bool()
        msg.data = self.enabled
        self.enable_pub.publish(msg)
        status = "ENABLED" if self.enabled else "DISABLED"
        self.get_logger().info(f'Controller {status}')

    def print_status(self):
        """Print current status to terminal."""
        status = "ENABLED" if self.enabled else "DISABLED"
        print(USAGE_MSG.format(
            linear_speed=self.linear_speed,
            angular_speed=self.angular_speed,
            status=status
        ))

    def run(self):
        """Main loop for keyboard input processing."""
        # Save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

        try:
            # Set terminal to raw mode for single-key input
            tty.setraw(sys.stdin.fileno())

            # Print initial status
            self.print_status()

            while rclpy.ok():
                # Process ROS callbacks
                rclpy.spin_once(self, timeout_sec=0)

                # Read keyboard input
                key = self.get_key(timeout=0.05)

                if key == '':
                    continue

                # Handle escape key
                if key == '\x1b':
                    self.get_logger().info('Exiting keyboard teleop...')
                    break

                # Handle Ctrl+C
                if key == '\x03':
                    break

                key_lower = key.lower()

                # Movement keys
                if key_lower in MOVE_BINDINGS:
                    x, y, z, az = MOVE_BINDINGS[key_lower]
                    self.linear_x = x
                    self.linear_y = y
                    self.linear_z = z
                    self.angular_z = az

                # Speed adjustment
                elif key_lower in SPEED_BINDINGS:
                    lin_mult, ang_mult = SPEED_BINDINGS[key_lower]
                    self.linear_speed *= lin_mult
                    self.angular_speed *= ang_mult
                    self.linear_speed = max(0.1, min(5.0, self.linear_speed))
                    self.angular_speed = max(0.1, min(3.0, self.angular_speed))

                # Stop (space bar)
                elif key == ' ':
                    self.linear_x = 0.0
                    self.linear_y = 0.0
                    self.linear_z = 0.0
                    self.angular_z = 0.0

                # Toggle enable
                elif key_lower == 't':
                    self.toggle_enable()

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

        finally:
            # Stop the drone before exiting
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.linear_z = 0.0
            self.angular_z = 0.0
            self.timer_callback()  # Publish stop command

            # Restore terminal settings
            if self.settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    """Entry point for keyboard teleop node."""
    print("Starting X500 Depth Keyboard Teleop...")
    print("Press 'T' to enable controller, then use WASD/RF/QE to control.")
    print("Press 'Esc' to exit.\n")

    rclpy.init(args=args)
    node = KeyboardTeleopNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
