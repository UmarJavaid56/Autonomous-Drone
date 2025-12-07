#include <memory>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class VioSITLTestNode : public rclcpp::Node
{
public:
    VioSITLTestNode()
    : Node("vio_sitl_test")
    {
        pose_topic_ = this->declare_parameter<std::string>(
            "pose_topic",
            "/orbslam3/pose"
        );
        
        udp_ip_ = this->declare_parameter<std::string>(
            "udp_ip",
            "127.0.0.1"
        );
        
        udp_port_ = this->declare_parameter<int>(
            "udp_port",
            14550
        );

        RCLCPP_INFO(this->get_logger(),
                    "Subscribing to pose topic: %s",
                    pose_topic_.c_str());
        
        RCLCPP_INFO(this->get_logger(),
                    "UDP target: %s:%d",
                    udp_ip_.c_str(), udp_port_);

        // Create UDP socket
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
        }

        // Setup server address
        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(udp_port_);
        inet_pton(AF_INET, udp_ip_.c_str(), &server_addr_.sin_addr);

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_,
            10,
            std::bind(&VioSITLTestNode::poseCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "VIO SITL Test node started");
    }

    ~VioSITLTestNode()
    {
        if (udp_socket_ >= 0) {
            close(udp_socket_);
        }
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        const auto &p = msg->pose.position;
        const auto &q = msg->pose.orientation;

        RCLCPP_INFO(this->get_logger(),
                    "ORB-SLAM3 pose: "
                    "pos [x=%.3f y=%.3f z=%.3f], "
                    "ori [x=%.3f y=%.3f z=%.3f w=%.3f]",
                    p.x, p.y, p.z,
                    q.x, q.y, q.z, q.w);

        // Send pose data over UDP
        char buffer[256];
        snprintf(buffer, sizeof(buffer),
                 "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                 p.x, p.y, p.z, q.x, q.y, q.z, q.w);

        sendto(udp_socket_, buffer, strlen(buffer), 0,
               (struct sockaddr*)&server_addr_, sizeof(server_addr_));
    }

    std::string pose_topic_;
    std::string udp_ip_;
    int udp_port_;
    int udp_socket_;
    struct sockaddr_in server_addr_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VioSITLTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

