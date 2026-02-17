/**
 * Point cloud filter: removes NaN/Inf and optional ground points.
 * Use this so RTAB-Map (libpointmatcher) and ESDF server get clean data and
 * don't hit "invalid data: normal=-nan -nan -nan" or topic errors.
 */

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace esdf_server
{

class PointCloudFilterNode : public rclcpp::Node
{
public:
  explicit PointCloudFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("point_cloud_filter", options)
  {
    declare_parameter<std::string>("input_topic", "/oak_d_lite/depth/points");
    declare_parameter<std::string>("output_topic", "/oak_d_lite/depth/points_filtered");
    declare_parameter<double>("min_z_ground", 0.02);  // drop points at or below this (sensor frame) to avoid ground NaNs
    declare_parameter<bool>("filter_ground", true);

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    min_z_ground_ = get_parameter("min_z_ground").as_double();
    filter_ground_ = get_parameter("filter_ground").as_bool();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, 10, std::bind(&PointCloudFilterNode::callback, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

    RCLCPP_INFO(get_logger(), "Point cloud filter: %s -> %s (filter_ground=%s, min_z=%.3f)",
      input_topic_.c_str(), output_topic_.c_str(), filter_ground_ ? "true" : "false", min_z_ground_);
  }

private:
  static bool valid(float v)
  {
    return std::isfinite(v) && !std::isnan(v);
  }

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->header.frame_id.empty()) {
      return;
    }
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, cloud);

    pcl::PointCloud<pcl::PointXYZ> out;
    out.header = cloud.header;
    out.reserve(cloud.size());
    int dropped = 0;
    for (const auto & pt : cloud.points) {
      if (!valid(pt.x) || !valid(pt.y) || !valid(pt.z)) {
        dropped++;
        continue;
      }
      if (filter_ground_ && pt.z < min_z_ground_) {
        dropped++;
        continue;
      }
      out.points.push_back(pt);
    }
    out.width = static_cast<std::uint32_t>(out.points.size());
    out.height = 1;
    out.is_dense = true;

    if (out.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Filtered cloud empty (dropped %d points); not publishing.", dropped);
      return;
    }
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(out, out_msg);
    out_msg.header = msg->header;
    pub_->publish(out_msg);
  }

  std::string input_topic_;
  std::string output_topic_;
  double min_z_ground_;
  bool filter_ground_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace esdf_server

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<esdf_server::PointCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}
