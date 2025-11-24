#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <System.h>           // ORB_SLAM3
#include <sophus/se3.hpp>     // Sophus::SE3f
#include <Eigen/Core>
#include <Eigen/Geometry>

class ORBMonoNode : public rclcpp::Node
{
public:
    ORBMonoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("orbslam3_mono_node", options)
    {
        // Declare parameters
        this->declare_parameter<std::string>("voc_file", "");
        this->declare_parameter<std::string>("settings_file", "");
        this->declare_parameter<std::string>("image_topic", "/camera/image_raw");

        std::string voc_file      = this->get_parameter("voc_file").as_string();
        std::string settings_file = this->get_parameter("settings_file").as_string();
        image_topic_              = this->get_parameter("image_topic").as_string();

        if (voc_file.empty() || settings_file.empty()) {
            RCLCPP_FATAL(
                this->get_logger(),
                "Parameters 'voc_file' and 'settings_file' must be set");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Starting ORB-SLAM3 MONO with voc: '%s', settings: '%s', topic: '%s'",
            voc_file.c_str(), settings_file.c_str(), image_topic_.c_str());

        // bUseViewer = false to avoid Pangolin/OpenGL on headless Jetson
        slam_ = std::make_unique<ORB_SLAM3::System>(
            voc_file, settings_file, ORB_SLAM3::System::MONOCULAR, false);

        // Pose publisher
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/orbslam3/pose", 10);

        // Subscribe to the image topic
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_, 10,
            std::bind(&ORBMonoNode::imageCallback, this, std::placeholders::_1));
    }

    ~ORBMonoNode() override
    {
        if (slam_) {
            slam_->Shutdown();
        }
    }

private:
    // Convert Sophus::SE3f Tcw (world -> camera) to Pose in world frame
    bool TcwToPose(const Sophus::SE3f &Tcw, geometry_msgs::msg::Pose &pose_msg)
    {
        // Rotation and translation in float
        Eigen::Matrix3f Rcw = Tcw.so3().matrix();
        Eigen::Vector3f tcw = Tcw.translation();

        // Invert to get world->camera -> camera in world frame
        Eigen::Matrix3f Rwc = Rcw.transpose();
        Eigen::Vector3f twc = -Rwc * tcw;

        // Rotation matrix -> quaternion
        Eigen::Quaternionf q(Rwc);
        q.normalize();

        pose_msg.position.x = twc.x();
        pose_msg.position.y = twc.y();
        pose_msg.position.z = twc.z();

        pose_msg.orientation.w = q.w();
        pose_msg.orientation.x = q.x();
        pose_msg.orientation.y = q.y();
        pose_msg.orientation.z = q.z();

        return true;
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            // Our OAK publisher sends BGR8
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat imRGB;
        cv::cvtColor(cv_ptr->image, imRGB, cv::COLOR_BGR2RGB);

        double tframe = msg->header.stamp.sec +
                        msg->header.stamp.nanosec * 1e-9;

        // ORB-SLAM3 (newer) returns Sophus::SE3f
        Sophus::SE3f Tcw = slam_->TrackMonocular(imRGB, tframe);

        // (Optionally you could check tracking state here via slam_->GetTrackingState())

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = msg->header.stamp;
        pose_msg.header.frame_id = "world";   // or "map"

        if (!TcwToPose(Tcw, pose_msg.pose)) {
            return;
        }

        pose_pub_->publish(pose_msg);
    }

    std::unique_ptr<ORB_SLAM3::System> slam_;
    std::string image_topic_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ORBMonoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

