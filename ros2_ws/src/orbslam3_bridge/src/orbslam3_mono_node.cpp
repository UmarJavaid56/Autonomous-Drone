#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/core/core.hpp>

#include <System.h>   // ORB_SLAM3

using std::placeholders::_1;

class OrbSlam3MonoNode : public rclcpp::Node
{
public:
  OrbSlam3MonoNode()
  : Node("orbslam3_mono_node")
  {
    // Parameters for vocab + settings
    this->declare_parameter<std::string>("voc_file", "/home/group7/Desktop/Autonomous-Drone/ros2_ws/src/external/ORB_SLAM3/Vocabulary/ORBvoc.txt");
    this->declare_parameter<std::string>("settings_file", "/home/group7/Desktop/Autonomous-Drone/ros2_ws/src/external/ORB_SLAM3/Examples/Monocular/YourCamera.yaml");
    this->declare_parameter<std::string>("image_topic", "/camera/image_raw");

    std::string voc_file, settings_file, image_topic;
    this->get_parameter("voc_file", voc_file);
    this->get_parameter("settings_file", settings_file);
    this->get_parameter("image_topic", image_topic);

    RCLCPP_INFO(get_logger(), "ORB-SLAM3 vocab: %s", voc_file.c_str());
    RCLCPP_INFO(get_logger(), "ORB-SLAM3 settings: %s", settings_file.c_str());
    RCLCPP_INFO(get_logger(), "Subscribing to: %s", image_topic.c_str());

    // Initialize ORB-SLAM3 system (monocular)
    slam_ = std::make_unique<ORB_SLAM3::System>(
      voc_file,
      settings_file,
      ORB_SLAM3::System::MONOCULAR,
      true    // use viewer
    );

    // Subscribe to image topic
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic,
      10,
      std::bind(&OrbSlam3MonoNode::imageCallback, this, _1)
    );
  }

  ~OrbSlam3MonoNode() override
  {
    if (slam_) {
      slam_->Shutdown();
    }
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;

    try {
      // Try to interpret as grayscale first
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &)
    {
      // Fallback to color
      try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
    }

    double tframe = rclcpp::Time(msg->header.stamp).seconds();

    slam_->TrackMonocular(cv_ptr->image, tframe);
  }

  std::unique_ptr<ORB_SLAM3::System> slam_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OrbSlam3MonoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

