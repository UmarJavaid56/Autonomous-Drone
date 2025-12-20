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

        // Publisher for MAVROS vision pose topic
        vision_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/vision_pose/pose",
            10
        );

        RCLCPP_INFO(this->get_logger(), "vio_mavlink_bridge node started");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /mavros/vision_pose/pose");
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        const auto &p = msg->pose.position;
        const auto &q = msg->pose.orientation;

        RCLCPP_DEBUG(this->get_logger(),
                    "Received pose: "
                    "pos [x=%.3f y=%.3f z=%.3f], "
                    "ori [x=%.3f y=%.3f z=%.3f w=%.3f]",
                    p.x, p.y, p.z,
                    q.x, q.y, q.z, q.w);

        // Republish to MAVROS vision pose topic
        // Note: Assuming input data is already in NED frame for testing
        auto vision_pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
        vision_pose_msg->header = msg->header;
        vision_pose_msg->header.frame_id = "map";  // MAVROS expects map frame
        vision_pose_msg->pose = msg->pose;

        vision_pose_pub_->publish(*vision_pose_msg);
    }

    std::string pose_topic_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VioMavlinkBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

