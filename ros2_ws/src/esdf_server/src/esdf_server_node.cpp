/**
 * ESDF Server node (OctoMap fallback backend).
 *
 * Subscribes to PointCloud2 (e.g. from RTAB-Map or depth camera), transforms to map frame,
 * builds a volumetric representation and answers GetDistance requests for collision checking.
 * Uses a voxelized point cloud + KdTree for distance-to-obstacle queries (ESDF-like).
 * Planner depends only on the GetDistance interface so backends can be swapped to NVBlox/Voxblox.
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <esdf_msgs/srv/get_distance.hpp>

namespace esdf_server
{

class EsdfServerNode : public rclcpp::Node
{
public:
  explicit EsdfServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("esdf_server", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("point_cloud_topic", "/oak_d_lite/depth/points");
    declare_parameter<std::string>("map_frame_id", "map");
    declare_parameter<double>("voxel_size", 0.05);
    declare_parameter<double>("max_range", 5.0);
    declare_parameter<double>("min_range", 0.4);
    declare_parameter<double>("x_min", -10.0);
    declare_parameter<double>("x_max", 10.0);
    declare_parameter<double>("y_min", -10.0);
    declare_parameter<double>("y_max", 10.0);
    declare_parameter<double>("z_min", -1.0);
    declare_parameter<double>("z_max", 3.0);
    declare_parameter<bool>("publish_esdf_slice", true);
    declare_parameter<double>("esdf_slice_height", 0.0);
    declare_parameter<double>("esdf_slice_thickness", 3.0);  // 0 = thin slice at height; >0 = show voxels from height to height+thickness
    declare_parameter<bool>("accumulate_map", true);  // true = merge new scans into persistent map (for planning); false = live scan only

    point_cloud_topic_ = get_parameter("point_cloud_topic").as_string();
    map_frame_id_ = get_parameter("map_frame_id").as_string();
    voxel_size_ = get_parameter("voxel_size").as_double();
    max_range_ = get_parameter("max_range").as_double();
    min_range_ = get_parameter("min_range").as_double();
    x_min_ = get_parameter("x_min").as_double();
    x_max_ = get_parameter("x_max").as_double();
    y_min_ = get_parameter("y_min").as_double();
    y_max_ = get_parameter("y_max").as_double();
    z_min_ = get_parameter("z_min").as_double();
    z_max_ = get_parameter("z_max").as_double();
    publish_esdf_slice_ = get_parameter("publish_esdf_slice").as_bool();
    esdf_slice_height_ = get_parameter("esdf_slice_height").as_double();
    esdf_slice_thickness_ = get_parameter("esdf_slice_thickness").as_double();
    accumulate_map_ = get_parameter("accumulate_map").as_bool();

    get_distance_srv_ = create_service<esdf_msgs::srv::GetDistance>(
      "get_distance",
      std::bind(&EsdfServerNode::handleGetDistance, this, std::placeholders::_1, std::placeholders::_2));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic_, 10, std::bind(&EsdfServerNode::cloudCallback, this, std::placeholders::_1));

    if (publish_esdf_slice_) {
      esdf_slice_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("esdf_slice", 1);
    }

    RCLCPP_INFO(get_logger(), "ESDF server (OctoMap fallback): point_cloud=%s, frame=%s, voxel=%.3f, accumulate_map=%s",
      point_cloud_topic_.c_str(), map_frame_id_.c_str(), voxel_size_, accumulate_map_ ? "true" : "false");
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->header.frame_id.empty()) {
      return;
    }
    try {
      geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
        map_frame_id_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.5));

      sensor_msgs::msg::PointCloud2 cloud_transformed_msg;
      tf2::doTransform(*msg, cloud_transformed_msg, transform);

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(cloud_transformed_msg, *cloud_transformed);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto & pt : cloud_transformed->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      double d = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      if (d < min_range_ || d > max_range_) continue;
      if (pt.x < x_min_ || pt.x > x_max_ || pt.y < y_min_ || pt.y > y_max_ || pt.z < z_min_ || pt.z > z_max_) continue;
      filtered->points.push_back(pt);
    }

    if (filtered->empty()) {
      return;
    }

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(filtered);
    voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    voxel.filter(*downsampled);

    {
      std::lock_guard<std::mutex> lock(kdtree_mutex_);
      if (accumulate_map_ && obstacle_cloud_ && !obstacle_cloud_->empty()) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr combined(new pcl::PointCloud<pcl::PointXYZ>);
        *combined = *obstacle_cloud_ + *downsampled;
        pcl::VoxelGrid<pcl::PointXYZ> voxel_merge;
        voxel_merge.setInputCloud(combined);
        voxel_merge.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_merge.filter(*merged);
        obstacle_cloud_ = merged;
      } else {
        obstacle_cloud_ = downsampled;
      }
      kdtree_.setInputCloud(obstacle_cloud_);
      map_ready_ = obstacle_cloud_->size() > 0;
    }

    if (publish_esdf_slice_ && esdf_slice_pub_) {
      publishEsdfSlice();
    }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "ESDF: TF lookup failed: %s", ex.what());
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "ESDF: cloud callback error: %s", ex.what());
    }
  }

  void handleGetDistance(
    const std::shared_ptr<esdf_msgs::srv::GetDistance::Request> request,
    std::shared_ptr<esdf_msgs::srv::GetDistance::Response> response)
  {
    double x = request->x;
    double y = request->y;
    double z = request->z;

    if (x < x_min_ || x > x_max_ || y < y_min_ || y > y_max_ || z < z_min_ || z > z_max_) {
      response->valid = false;
      response->distance = 0.0;
      return;
    }

    std::lock_guard<std::mutex> lock(kdtree_mutex_);
    if (!map_ready_ || !obstacle_cloud_ || obstacle_cloud_->empty()) {
      response->valid = false;
      response->distance = 0.0;
      return;
    }

    pcl::PointXYZ query(x, y, z);
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    if (kdtree_.nearestKSearch(query, 1, indices, distances) > 0) {
      response->distance = std::sqrt(distances[0]);
      response->valid = true;
      // Optional: treat inside obstacle as negative (we don't have true SDF, so keep positive and let planner use safety_margin)
    } else {
      response->valid = false;
      response->distance = 0.0;
    }
  }

  void publishEsdfSlice()
  {
    std::lock_guard<std::mutex> lock(kdtree_mutex_);
    if (!obstacle_cloud_ || obstacle_cloud_->empty()) return;

    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_id_;
    m.header.stamp = now();
    m.ns = "esdf_slice";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = voxel_size_;
    m.scale.y = voxel_size_;
    m.scale.z = voxel_size_;
    m.color.a = 0.6f;
    m.color.r = 0.2f;
    m.color.g = 0.6f;
    m.color.b = 1.0f;

    const double half_band = voxel_size_ * 2;
    const double z_min_viz = esdf_slice_thickness_ > 0.0 ? esdf_slice_height_ : (esdf_slice_height_ - half_band);
    const double z_max_viz = esdf_slice_thickness_ > 0.0 ? (esdf_slice_height_ + esdf_slice_thickness_) : (esdf_slice_height_ + half_band);
    for (const auto & pt : obstacle_cloud_->points) {
      if (pt.z < z_min_viz || pt.z > z_max_viz) continue;
      geometry_msgs::msg::Point p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      m.points.push_back(p);
    }
    if (!m.points.empty()) {
      ma.markers.push_back(m);
      esdf_slice_pub_->publish(ma);
    }
  }

  std::string point_cloud_topic_;
  std::string map_frame_id_;
  double voxel_size_;
  double max_range_;
  double min_range_;
  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
  bool publish_esdf_slice_;
  double esdf_slice_height_;
  double esdf_slice_thickness_;
  bool accumulate_map_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Service<esdf_msgs::srv::GetDistance>::SharedPtr get_distance_srv_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr esdf_slice_pub_;

  std::mutex kdtree_mutex_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
  bool map_ready_{false};
};

}  // namespace esdf_server

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<esdf_server::EsdfServerNode>());
  rclcpp::shutdown();
  return 0;
}
