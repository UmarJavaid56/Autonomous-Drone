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
            "/orbslam3/pose_processed"   // <-- subscribe to processed ORB-SLAM3 output
        );

        RCLCPP_INFO(this->get_logger(),
                    "VIO bridge subscribing to pose topic: %s",
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

        // Convert from ENU to NED frame for PX4
        // ENU: X=East, Y=North, Z=Up
        // NED: X=North, Y=East, Z=Down
        auto vision_pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
        vision_pose_msg->header = msg->header;
        vision_pose_msg->header.frame_id = "map";  // MAVROS expects map frame

        // ENU → NED conversion
        vision_pose_msg->pose.position.x = p.y;    // ENU Y (North) → NED X (North)
        vision_pose_msg->pose.position.y = p.x;    // ENU X (East) → NED Y (East)
        vision_pose_msg->pose.position.z = -p.z;   // ENU Z (Up) → NED Z (Down) = -Up

        // Orientation also needs to be transformed for the frame change
        // For now, keep orientation as-is (assuming it's relative to ENU frame)
        // TODO: Implement proper quaternion transformation for ENU→NED
        vision_pose_msg->pose.orientation = msg->pose.orientation;

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

