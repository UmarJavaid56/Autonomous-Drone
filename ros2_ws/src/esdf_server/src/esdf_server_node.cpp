/**
 * ESDF Server node (OctoMap fallback backend).
 *
 * Subscribes to PointCloud2 (e.g. from RTAB-Map or depth camera), transforms to map frame,
 * builds a volumetric representation and answers GetDistance requests for collision checking.
 * Uses a voxelized point cloud + KdTree for distance-to-obstacle queries (ESDF-like).
 * Planner depends only on the GetDistance interface so backends can be swapped to NVBlox/Voxblox.
 */

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
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

struct VoxelKey
{
  int x;
  int y;
  int z;

  bool operator==(const VoxelKey & other) const noexcept
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & k) const noexcept
  {
    const std::size_t h1 = std::hash<int>{}(k.x);
    const std::size_t h2 = std::hash<int>{}(k.y);
    const std::size_t h3 = std::hash<int>{}(k.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

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
    declare_parameter<double>("max_map_age_sec", 2.0);
    declare_parameter<bool>("publish_esdf_slice", true);
    declare_parameter<double>("esdf_slice_height", 0.0);
    declare_parameter<double>("esdf_slice_thickness", 3.0);  // 0 = thin slice at height; >0 = show voxels from height to height+thickness
    declare_parameter<bool>("accumulate_map", true);  // true = merge new scans into persistent map (for planning); false = live scan only
    declare_parameter<bool>("unknown_is_occupied", true);
    declare_parameter<int>("max_raycast_points_per_cloud", 300);
    declare_parameter<int>("raycast_decimation", 4);
    declare_parameter<double>("raycast_step_m", 0.10);
    declare_parameter<int>("max_observed_voxels", 1200000);

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
    max_map_age_sec_ = get_parameter("max_map_age_sec").as_double();
    publish_esdf_slice_ = get_parameter("publish_esdf_slice").as_bool();
    esdf_slice_height_ = get_parameter("esdf_slice_height").as_double();
    esdf_slice_thickness_ = get_parameter("esdf_slice_thickness").as_double();
    accumulate_map_ = get_parameter("accumulate_map").as_bool();
    unknown_is_occupied_ = get_parameter("unknown_is_occupied").as_bool();
    max_raycast_points_per_cloud_ = std::max(
      1, static_cast<int>(get_parameter("max_raycast_points_per_cloud").as_int()));
    raycast_decimation_ = std::max(
      1, static_cast<int>(get_parameter("raycast_decimation").as_int()));
    raycast_step_m_ = std::max(1e-3, get_parameter("raycast_step_m").as_double());
    max_observed_voxels_ = std::max(
      10000, static_cast<int>(get_parameter("max_observed_voxels").as_int()));

    // Service in a reentrant callback group so it can run concurrently with cloud processing
    srv_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    get_distance_srv_ = create_service<esdf_msgs::srv::GetDistance>(
      "get_distance",
      std::bind(&EsdfServerNode::handleGetDistance, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      srv_cb_group_);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic_, 10, std::bind(&EsdfServerNode::cloudCallback, this, std::placeholders::_1));

    if (publish_esdf_slice_) {
      esdf_slice_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("esdf_slice", 1);
    }

    RCLCPP_INFO(
      get_logger(),
      "ESDF server (OctoMap fallback): point_cloud=%s, frame=%s, voxel=%.3f, "
      "accumulate_map=%s, unknown_is_occupied=%s",
      point_cloud_topic_.c_str(), map_frame_id_.c_str(), voxel_size_,
      accumulate_map_ ? "true" : "false", unknown_is_occupied_ ? "true" : "false");
  }

private:
  bool inBounds(double x, double y, double z) const
  {
    return x >= x_min_ && x <= x_max_ &&
           y >= y_min_ && y <= y_max_ &&
           z >= z_min_ && z <= z_max_;
  }

  VoxelKey pointToVoxel(double x, double y, double z) const
  {
    return VoxelKey{
      static_cast<int>(std::floor(x / voxel_size_)),
      static_cast<int>(std::floor(y / voxel_size_)),
      static_cast<int>(std::floor(z / voxel_size_))};
  }

  void markObservedFree(double x, double y, double z)
  {
    if (!inBounds(x, y, z)) {
      return;
    }
    const auto key = pointToVoxel(x, y, z);
    if (observed_occupied_voxels_.find(key) != observed_occupied_voxels_.end()) {
      return;
    }
    observed_free_voxels_.insert(key);
  }

  void markObservedOccupied(double x, double y, double z)
  {
    if (!inBounds(x, y, z)) {
      return;
    }
    const auto key = pointToVoxel(x, y, z);
    observed_occupied_voxels_.insert(key);
    observed_free_voxels_.erase(key);
  }

  void updateObservedSpace(
    double sensor_x, double sensor_y, double sensor_z,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & downsampled)
  {
    if (!unknown_is_occupied_ || !downsampled || downsampled->empty()) {
      return;
    }

    ++cloud_counter_;
    if ((cloud_counter_ % static_cast<std::uint64_t>(raycast_decimation_)) != 0) {
      return;
    }

    if (!accumulate_map_) {
      observed_free_voxels_.clear();
      observed_occupied_voxels_.clear();
    }

    markObservedFree(sensor_x, sensor_y, sensor_z);

    const std::size_t max_points = static_cast<std::size_t>(max_raycast_points_per_cloud_);
    const std::size_t stride = std::max<std::size_t>(
      1, (downsampled->points.size() + max_points - 1) / max_points);
    const double ray_step = std::max(0.5 * voxel_size_, raycast_step_m_);

    for (std::size_t i = 0; i < downsampled->points.size(); i += stride) {
      const auto & pt = downsampled->points[i];
      const double dx = pt.x - sensor_x;
      const double dy = pt.y - sensor_y;
      const double dz = pt.z - sensor_z;
      const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      const int steps = std::max(1, static_cast<int>(dist / ray_step));

      // Mark cells along the ray as observed free (excluding the hit point).
      for (int s = 0; s < steps; ++s) {
        const double t = static_cast<double>(s) / static_cast<double>(steps);
        markObservedFree(sensor_x + t * dx, sensor_y + t * dy, sensor_z + t * dz);
      }
      // Endpoint is an obstacle observation.
      markObservedOccupied(pt.x, pt.y, pt.z);
    }

    const auto observed_total = observed_free_voxels_.size() + observed_occupied_voxels_.size();
    if (observed_total > static_cast<std::size_t>(max_observed_voxels_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ESDF observed-space cache exceeded %d voxels (%zu), clearing cache.",
        max_observed_voxels_, observed_total);
      observed_free_voxels_.clear();
      observed_occupied_voxels_.clear();
    }
  }

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

    const double sensor_x = transform.transform.translation.x;
    const double sensor_y = transform.transform.translation.y;
    const double sensor_z = transform.transform.translation.z;

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto & pt : cloud_transformed->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      // Range gate in sensor-centric coordinates; using map origin here would
      // incorrectly drop points as the drone moves away from (0,0,0).
      const double dx = pt.x - sensor_x;
      const double dy = pt.y - sensor_y;
      const double dz = pt.z - sensor_z;
      const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
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
        if (!accumulate_map_) {
          observed_free_voxels_.clear();
          observed_occupied_voxels_.clear();
        }
      }
      updateObservedSpace(sensor_x, sensor_y, sensor_z, downsampled);
      kdtree_.setInputCloud(obstacle_cloud_);
      map_ready_ = obstacle_cloud_->size() > 0;
      if (map_ready_) {
        last_map_update_time_ = now();
      }
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

    if (!inBounds(x, y, z)) {
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
    if (max_map_age_sec_ > 0.0) {
      const double age = (now() - last_map_update_time_).seconds();
      if (age > max_map_age_sec_) {
        response->valid = false;
        response->distance = 0.0;
        return;
      }
    }

    if (unknown_is_occupied_) {
      const auto query_key = pointToVoxel(x, y, z);
      if (observed_occupied_voxels_.find(query_key) != observed_occupied_voxels_.end()) {
        response->valid = true;
        response->distance = 0.0;
        return;
      }
      if (observed_free_voxels_.find(query_key) == observed_free_voxels_.end()) {
        response->valid = false;
        response->distance = 0.0;
        return;
      }
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
  double max_map_age_sec_;
  bool publish_esdf_slice_;
  double esdf_slice_height_;
  double esdf_slice_thickness_;
  bool accumulate_map_;
  bool unknown_is_occupied_;
  int max_raycast_points_per_cloud_;
  int raycast_decimation_;
  double raycast_step_m_;
  int max_observed_voxels_;
  std::uint64_t cloud_counter_{0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::CallbackGroup::SharedPtr srv_cb_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Service<esdf_msgs::srv::GetDistance>::SharedPtr get_distance_srv_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr esdf_slice_pub_;

  std::mutex kdtree_mutex_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
  std::unordered_set<VoxelKey, VoxelKeyHash> observed_free_voxels_;
  std::unordered_set<VoxelKey, VoxelKeyHash> observed_occupied_voxels_;
  bool map_ready_{false};
  rclcpp::Time last_map_update_time_;
};

}  // namespace esdf_server

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<esdf_server::EsdfServerNode>();
  // Multi-threaded executor so service requests can be handled while point cloud
  // processing is in progress (prevents service timeouts from the RRT* planner).
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
