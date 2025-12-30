#!/usr/bin/env python3
"""
MAVLink VIO Monitor - Checks if PX4 is receiving VISION_POSITION_ESTIMATE messages
"""

import socket
import time
import struct
from pymavlink import mavutil

class MAVLinkVIOMonitor:
    def __init__(self):
        self.mav = None
        self.connected = False
        self.last_vision_time = 0
        self.vision_count = 0
        self.start_time = time.time()

    def connect(self):
        """Connect to PX4 SITL MAVLink"""
        try:
            # Connect to PX4 SITL onboard MAVLink port
            self.mav = mavutil.mavlink_connection('udp:127.0.0.1:14580', source_system=255, source_component=0)
            self.mav.wait_heartbeat(timeout=5)
            print("✅ Connected to PX4 SITL MAVLink")
            self.connected = True
            return True
        except Exception as e:
            print(f"❌ Failed to connect to PX4 MAVLink: {e}")
            return False

    def monitor_vision_messages(self):
        """Monitor for VISION_POSITION_ESTIMATE messages"""
        print("Listening for VISION_POSITION_ESTIMATE messages...")
        print("Press Ctrl+C to stop")

        try:
            while True:
                # Receive MAVLink messages
                msg = self.mav.recv_match(blocking=True, timeout=1.0)

                if msg:
                    if msg.get_type() == 'VISION_POSITION_ESTIMATE':
                        self.vision_count += 1
                        self.last_vision_time = time.time()

                        # Extract position data
                        x, y, z = msg.x, msg.y, msg.z
                        print(f"📍 VISION_POSITION_ESTIMATE #{self.vision_count}: "
                              f"X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

                        # Check covariance if available
                        if hasattr(msg, 'covariance'):
                            cov = msg.covariance
                            if len(cov) >= 9:
                                pos_cov = [cov[0], cov[4], cov[8]]  # diagonal elements
                                print(f"   Covariance: X={pos_cov[0]:.6f}, Y={pos_cov[1]:.6f}, Z={pos_cov[2]:.6f}")

                    elif msg.get_type() == 'HEARTBEAT':
                        # Just log heartbeat occasionally
                        pass

                # Periodic status report
                current_time = time.time()
                if current_time - self.start_time >= 5.0:
                    uptime = current_time - self.start_time
                    rate = self.vision_count / uptime if uptime > 0 else 0
                    age = current_time - self.last_vision_time if self.last_vision_time > 0 else float('inf')

                    print(f"\n--- Status Report ({uptime:.1f}s) ---")
                    print(f"Vision messages received: {self.vision_count}")
                    print(f"Average rate: {rate:.1f} Hz")
                    print(f"Last message age: {age:.1f}s")

                    if age > 2.0:
                        print("⚠️  WARNING: No vision messages in last 2 seconds!")
                    else:
                        print("✅ Vision data flowing normally")

                    print("-" * 30)
                    self.start_time = current_time

        except KeyboardInterrupt:
            print("\nStopping MAVLink monitor...")
        except Exception as e:
            print(f"Error monitoring MAVLink: {e}")

    def run(self):
        if not self.connect():
            return

        self.monitor_vision_messages()

if __name__ == '__main__':
    monitor = MAVLinkVIOMonitor()
    monitor.run()

