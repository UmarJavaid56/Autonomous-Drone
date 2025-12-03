#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class VioMavlinkBridgeNode : public rclcpp::Node
{
public:
    VioMavlinkBridgeNode()
    : Node("vio_mavlink_bridge")
    {
        // Declare a parameter for the pose topic, with a default
        pose_topic_ = this->declare_parameter<std::string>(
            "pose_topic",
            "/orbslam3/pose"   // <-- change this default if needed
        );

        RCLCPP_INFO(this->get_logger(),
                    "Subscribing to pose topic: %s",
                    pose_topic_.c_str());

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_,
            10,
            std::bind(&VioMavlinkBridgeNode::poseCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "vio_mavlink_bridge node started");
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

        // TODO: convert pose -> MAVLink ODOMETRY and send over serial
    }

    std::string pose_topic_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VioMavlinkBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

