#include <memory>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

// MAVLink constants
#define MAVLINK_STX 0xFD  // MAVLink 2.0 start byte
#define MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE 102

// MAVLink library handles CRC calculation automatically

class VioMavlinkDirectBridgeNode : public rclcpp::Node
{
public:
    VioMavlinkDirectBridgeNode()
    : Node("vio_mavlink_direct_bridge"), last_heartbeat_time_(0)
    {
        // Declare parameters
        pose_topic_ = this->declare_parameter<std::string>(
            "pose_topic",
            "/orbslam3/pose_processed"
        );

        mavlink_port_ = this->declare_parameter<int>(
            "mavlink_port",
            14580  // PX4 SITL onboard MAVLink port
        );

        mavlink_host_ = this->declare_parameter<std::string>(
            "mavlink_host",
            "127.0.0.1"
        );

        system_id_ = this->declare_parameter<uint8_t>(
            "system_id",
            1
        );

        component_id_ = this->declare_parameter<uint8_t>(
            "component_id",
            0  // Use component ID 0 (all) for testing
        );

        RCLCPP_INFO(this->get_logger(),
                    "VIO direct bridge subscribing to pose topic: %s",
                    pose_topic_.c_str());
        RCLCPP_INFO(this->get_logger(),
                    "Sending MAVLink to %s:%d (system_id: %d, component_id: %d)",
                    mavlink_host_.c_str(), mavlink_port_, system_id_, component_id_);
        // TEMP: Hardcode the values we want
        mavlink_port_ = 14580;
        component_id_ = 1;  // Camera component ID

        RCLCPP_INFO(this->get_logger(),
                    "Parameters: pose_topic=%s, mavlink_port=%d, component_id=%d",
                    pose_topic_.c_str(), mavlink_port_, component_id_);

        // Create pose subscriber
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_,
            10,
            std::bind(&VioMavlinkDirectBridgeNode::poseCallback, this, std::placeholders::_1)
        );

        // Create IMU subscriber
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw",
            10,
            std::bind(&VioMavlinkDirectBridgeNode::imuCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "VIO direct bridge subscribing to IMU topic: /imu/data_raw");

        // Initialize MAVLink socket
        initMavlinkSocket();

        // Send initial heartbeat to establish MAVLink communication
        sendMavlinkHeartbeat();

        RCLCPP_INFO(this->get_logger(), "VIO MAVLink direct bridge started");
    }

    ~VioMavlinkDirectBridgeNode()
    {
        if (sockfd_ >= 0) {
            close(sockfd_);
        }
    }

private:
    void initMavlinkSocket()
    {
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create MAVLink socket: %s", strerror(errno));
            return;
        }

        // Set socket to non-blocking
        int flags = fcntl(sockfd_, F_GETFL, 0);
        if (fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set socket non-blocking: %s", strerror(errno));
            close(sockfd_);
            sockfd_ = -1;
            return;
        }

        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(mavlink_port_);
        server_addr_.sin_addr.s_addr = inet_addr(mavlink_host_.c_str());

        RCLCPP_INFO(this->get_logger(), "MAVLink socket initialized: %s:%d (fd: %d)", mavlink_host_.c_str(), mavlink_port_, sockfd_);

        // Test socket connection
        uint8_t test_buf[1] = {0xFF};
        ssize_t test_sent = sendto(sockfd_, test_buf, 1, 0, (struct sockaddr*)&server_addr_, sizeof(server_addr_));
        if (test_sent < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket test failed: %s (errno: %d)", strerror(errno), errno);
        } else {
            RCLCPP_INFO(this->get_logger(), "Socket test successful (sent %zd bytes)", test_sent);
        }
    }

    void sendMavlinkHeartbeat()
    {
        if (sockfd_ < 0) return;

        // Construct MAVLink HEARTBEAT message (9 bytes payload)
        uint8_t buf[32];
        int buf_idx = 0;

        // HEARTBEAT payload
        uint8_t payload[9] = {0};
        payload[0] = 18;  // MAV_TYPE_ONBOARD_CONTROLLER
        payload[1] = 8;   // MAV_AUTOPILOT_INVALID
        payload[2] = 1;   // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        payload[6] = 4;   // MAV_STATE_ACTIVE
        payload[8] = 3;   // MAVLink version 2.0

        // Build MAVLink 2.0 message
        buf[buf_idx++] = MAVLINK_STX;              // STX
        buf[buf_idx++] = sizeof(payload);          // Payload length (9)
        buf[buf_idx++] = 0;                        // Incompat flags
        buf[buf_idx++] = 0;                        // Compat flags
        buf[buf_idx++] = seq_++;                   // Sequence
        if (seq_ > 255) seq_ = 0;
        buf[buf_idx++] = system_id_;               // System ID
        buf[buf_idx++] = component_id_;            // Component ID
        buf[buf_idx++] = 0;                        // HEARTBEAT message ID

        // Payload
        memcpy(&buf[buf_idx], payload, sizeof(payload));
        buf_idx += sizeof(payload);

        // CRC for HEARTBEAT (CRC extra = 50)
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < sizeof(payload); i++) {
            crc ^= payload[i] << 8;
            for (int j = 0; j < 8; j++) {
                if (crc & 0x8000) {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc ^= 0;  // Message ID
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
        crc ^= 50;  // CRC extra for HEARTBEAT

        buf[buf_idx++] = crc & 0xFF;
        buf[buf_idx++] = (crc >> 8) & 0xFF;

        int len = buf_idx;
        ssize_t sent = sendto(sockfd_, buf, len, 0, (struct sockaddr*)&server_addr_, sizeof(server_addr_));

        if (sent < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send MAVLink HEARTBEAT: %s (errno: %d)", strerror(errno), errno);
        } else if (sent != len) {
            RCLCPP_WARN(this->get_logger(), "Partial heartbeat send: expected %d bytes, sent %zd bytes", len, sent);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Sent MAVLink HEARTBEAT (len: %zd)", sent);
        }
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
    {
        if (sockfd_ < 0) {
            RCLCPP_WARN(this->get_logger(), "MAVLink socket not initialized");
            return;
        }

        // Send heartbeat every 1 second
        auto now = std::chrono::steady_clock::now();
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        if (now_ms - last_heartbeat_time_ > 1000) {
            sendMavlinkHeartbeat();
            last_heartbeat_time_ = now_ms;
        }

        const auto &p = pose_msg->pose.position;
        const auto &q = pose_msg->pose.orientation;

        RCLCPP_DEBUG(this->get_logger(),
                    "Received pose: pos [x=%.3f y=%.3f z=%.3f]",
                    p.x, p.y, p.z);

        // Convert from ENU to NED frame for PX4
        // ENU: X=East, Y=North, Z=Up
        // NED: X=North, Y=East, Z=Down
        float ned_x = p.y;    // ENU Y (North) → NED X (North)
        float ned_y = p.x;    // ENU X (East) → NED Y (East)
        float ned_z = -p.z;   // ENU Z (Up) → NED Z (Down) = -Up

        // Convert quaternion to Euler angles for MAVLink
        // For VISION_POSITION_ESTIMATE, we need roll, pitch, yaw
        float roll, pitch, yaw;
        quaternionToEuler(q.x, q.y, q.z, q.w, roll, pitch, yaw);

        // Get timestamp in microseconds
        uint64_t timestamp_us = pose_msg->header.stamp.sec * 1000000ULL + pose_msg->header.stamp.nanosec / 1000;

        // Construct MAVLink VISION_POSITION_ESTIMATE message (118 bytes)
        // Based on MAVLink 2.0 specification: https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE

        uint8_t buf[128];  // Buffer for MAVLink message
        int buf_idx = 0;

        // VISION_POSITION_ESTIMATE payload (118 bytes)
        uint8_t payload[118];
        memset(payload, 0, sizeof(payload));

        // Pack fields in little-endian format
        memcpy(&payload[0], &timestamp_us, 8);   // uint64_t usec
        memcpy(&payload[8], &ned_x, 4);          // float x
        memcpy(&payload[12], &ned_y, 4);         // float y
        memcpy(&payload[16], &ned_z, 4);         // float z
        memcpy(&payload[20], &roll, 4);          // float roll
        memcpy(&payload[24], &pitch, 4);         // float pitch
        memcpy(&payload[28], &yaw, 4);           // float yaw

        // Covariance matrix (21 floats for symmetric 6x6 matrix)
        // Diagonal elements for variances (lower values = higher confidence)
        float* cov = reinterpret_cast<float*>(&payload[32]);
        cov[0] = 0.01f;   // x variance
        cov[6] = 0.01f;   // y variance
        cov[11] = 0.01f;  // z variance
        cov[15] = 0.001f; // roll variance
        cov[18] = 0.001f; // pitch variance
        cov[20] = 0.001f; // yaw variance

        // uint16_t reset_counter
        uint16_t reset_counter = 0;
        memcpy(&payload[32 + 84], &reset_counter, 2);

        // Build MAVLink 2.0 message
        buf[buf_idx++] = MAVLINK_STX;                           // STX (0xFD)
        buf[buf_idx++] = sizeof(payload);                       // Payload length (118)
        buf[buf_idx++] = 0;                                     // Incompat flags
        buf[buf_idx++] = 0;                                     // Compat flags
        buf[buf_idx++] = seq_++;                                // Sequence
        if (seq_ > 255) seq_ = 0;
        buf[buf_idx++] = system_id_;                            // System ID
        buf[buf_idx++] = component_id_;                         // Component ID
        buf[buf_idx++] = MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE; // Message ID

        // Payload
        memcpy(&buf[buf_idx], payload, sizeof(payload));
        buf_idx += sizeof(payload);

        // CRC calculation (CRC16-CCITT)
        uint16_t crc = 0xFFFF;
        // CRC for payload
        for (size_t i = 0; i < sizeof(payload); i++) {
            crc ^= payload[i] << 8;
            for (int j = 0; j < 8; j++) {
                if (crc & 0x8000) {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }
        // CRC for message ID
        crc ^= MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
        // XOR with CRC extra for VISION_POSITION_ESTIMATE (0x8C)
        crc ^= 0x8C;

        // Checksum
        buf[buf_idx++] = crc & 0xFF;
        buf[buf_idx++] = (crc >> 8) & 0xFF;

        int len = buf_idx;

        ssize_t sent = sendto(sockfd_, buf, len, 0,
                            (struct sockaddr*)&server_addr_, sizeof(server_addr_));

        if (sent < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send MAVLink message: %s (errno: %d)", strerror(errno), errno);
            RCLCPP_ERROR(this->get_logger(), "Socket fd: %d, target: %s:%d", sockfd_, mavlink_host_.c_str(), mavlink_port_);
        } else if (sent != len) {
            RCLCPP_WARN(this->get_logger(), "Partial send: expected %d bytes, sent %zd bytes", len, sent);
        } else {
            RCLCPP_INFO(this->get_logger(), "Sent VISION_POSITION_ESTIMATE: x=%.3f, y=%.3f, z=%.3f (seq: %d, sent %zd bytes)",
                         ned_x, ned_y, ned_z, seq_-1, sent);
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        if (sockfd_ < 0) {
            RCLCPP_WARN(this->get_logger(), "MAVLink socket not initialized");
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Received IMU: accel [x=%.3f y=%.3f z=%.3f], gyro [x=%.3f y=%.3f z=%.3f]",
                    imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z,
                    imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);

        // Get accelerometer data
        float accel_x = imu_msg->linear_acceleration.x;
        float accel_y = imu_msg->linear_acceleration.y;
        float accel_z = imu_msg->linear_acceleration.z;

        // Get gyroscope data
        float gyro_x = imu_msg->angular_velocity.x;
        float gyro_y = imu_msg->angular_velocity.y;
        float gyro_z = imu_msg->angular_velocity.z;

        // Get temperature (not available from ROS IMU, use 25°C)
        float temperature = 25.0f;

        // Get timestamp in microseconds
        uint64_t timestamp_us = imu_msg->header.stamp.sec * 1000000ULL + imu_msg->header.stamp.nanosec / 1000;

        // Construct MAVLink HIGHRES_IMU message (62 bytes payload)
        // Based on MAVLink 2.0 specification: https://mavlink.io/en/messages/common.html#HIGHRES_IMU

        uint8_t buf[128];  // Buffer for MAVLink message
        int buf_idx = 0;

        // HIGHRES_IMU payload (62 bytes)
        uint8_t payload[62];
        memset(payload, 0, sizeof(payload));

        // Pack fields in little-endian format
        memcpy(&payload[0], &timestamp_us, 8);      // uint64_t time_usec
        memcpy(&payload[8], &accel_x, 4);           // float xacc
        memcpy(&payload[12], &accel_y, 4);          // float yacc
        memcpy(&payload[16], &accel_z, 4);          // float zacc
        memcpy(&payload[20], &gyro_x, 4);           // float xgyro
        memcpy(&payload[24], &gyro_y, 4);           // float ygyro
        memcpy(&payload[28], &gyro_z, 4);           // float zgyro
        // Skip xmag, ymag, zmag (24 bytes) - magnetometer not available
        memcpy(&payload[32 + 12], &temperature, 4); // float temperature (offset by 12 for skipped mag fields)

        // Build MAVLink 2.0 message
        buf[buf_idx++] = MAVLINK_STX;                              // STX (0xFD)
        buf[buf_idx++] = sizeof(payload);                          // Payload length (62)
        buf[buf_idx++] = 0;                                        // Incompat flags
        buf[buf_idx++] = 0;                                        // Compat flags
        buf[buf_idx++] = seq_++;                                   // Sequence
        if (seq_ > 255) seq_ = 0;
        buf[buf_idx++] = system_id_;                               // System ID
        buf[buf_idx++] = component_id_;                            // Component ID
        buf[buf_idx++] = 105;                                      // HIGHRES_IMU message ID

        // Payload
        memcpy(&buf[buf_idx], payload, sizeof(payload));
        buf_idx += sizeof(payload);

        // CRC calculation (CRC16-CCITT)
        uint16_t crc = 0xFFFF;
        // CRC for payload
        for (size_t i = 0; i < sizeof(payload); i++) {
            crc ^= payload[i] << 8;
            for (int j = 0; j < 8; j++) {
                if (crc & 0x8000) {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }
        // CRC for message ID
        crc ^= 105;  // HIGHRES_IMU message ID
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
        // XOR with CRC extra for HIGHRES_IMU (0x96)
        crc ^= 0x96;

        // Checksum
        buf[buf_idx++] = crc & 0xFF;
        buf[buf_idx++] = (crc >> 8) & 0xFF;

        int len = buf_idx;

        ssize_t sent = sendto(sockfd_, buf, len, 0,
                            (struct sockaddr*)&server_addr_, sizeof(server_addr_));

        if (sent < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send HIGHRES_IMU MAVLink message: %s (errno: %d)", strerror(errno), errno);
        } else if (sent != len) {
            RCLCPP_WARN(this->get_logger(), "Partial IMU send: expected %d bytes, sent %zd bytes", len, sent);
        } else {
            RCLCPP_INFO(this->get_logger(), "Sent HIGHRES_IMU: accel[%.3f,%.3f,%.3f] gyro[%.3f,%.3f,%.3f] (seq: %d, sent %zd bytes)",
                         accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, seq_-1, sent);
        }
    }

    void quaternionToEuler(float x, float y, float z, float w, float& roll, float& pitch, float& yaw)
    {
        // Convert quaternion to Euler angles (ZYX rotation order)
        float sinr_cosp = 2 * (w * x + y * z);
        float cosr_cosp = 1 - 2 * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        float sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = std::asin(sinp);

        float siny_cosp = 2 * (w * z + x * y);
        float cosy_cosp = 1 - 2 * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    std::string pose_topic_;
    int mavlink_port_;
    std::string mavlink_host_;
    uint8_t system_id_;
    uint8_t component_id_;
    uint8_t seq_;  // MAVLink sequence counter
    int64_t last_heartbeat_time_;  // Last heartbeat timestamp

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // MAVLink socket
    int sockfd_;
    struct sockaddr_in server_addr_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VioMavlinkDirectBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
