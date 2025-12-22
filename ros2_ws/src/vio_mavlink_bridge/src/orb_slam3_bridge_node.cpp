#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class OrbSlam3BridgeNode : public rclcpp::Node
{
public:
    OrbSlam3BridgeNode()
    : Node("orb_slam3_bridge")
    {
        // Subscribe to ORB-SLAM3 pose output (assuming ENU frame)
        orbslam_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/orbslam3/pose", 10,
            std::bind(&OrbSlam3BridgeNode::poseCallback, this, std::placeholders::_1));

        // Publisher for internal pose processing (same topic as VIO bridge expects)
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/orbslam3/pose_processed", 10);

        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 Bridge started - subscribing to /orbslam3/pose");
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // ORB-SLAM3 outputs in camera frame (typically ENU-like)

        // TODO: Add neural network processing here
        // - Pose validation using neural networks
        // - Outlier detection and filtering
        // - Fusion with other sensor data
        // - Confidence scoring
        // - Trajectory smoothing

        // For now, just forward the data
        pose_pub_->publish(*msg);

        RCLCPP_DEBUG(this->get_logger(), "Received ORB-SLAM3 pose");
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr orbslam_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OrbSlam3BridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
