/**
 * 3D RRT* global planner using OMPL and ESDF collision checking.
 * State space: (x, y, z, yaw). Runs asynchronously at ~1-3 Hz; does not block PX4 offboard.
 */

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <esdf_msgs/srv/get_distance.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/StateSpace.h>  // CompoundStateSpace
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/State.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/PathGeometric.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace rrt_star_planner
{

class EsdfStateValidityChecker : public ob::StateValidityChecker
{
public:
  EsdfStateValidityChecker(
    const rclcpp::Client<esdf_msgs::srv::GetDistance>::SharedPtr & client,
    const ob::SpaceInformationPtr & si,
    double drone_radius,
    double safety_margin)
  : ob::StateValidityChecker(si),
    client_(client),
    drone_radius_(drone_radius),
    safety_margin_(safety_margin)
  {}

  bool isValid(const ob::State * state) const override
  {
    auto * comp = state->as<ob::CompoundStateSpace::StateType>();
    const auto * pos = comp->as<ob::RealVectorStateSpace::StateType>(0);
    double x = (*pos)[0], y = (*pos)[1], z = (*pos)[2];

    auto request = std::make_shared<esdf_msgs::srv::GetDistance::Request>();
    request->x = x;
    request->y = y;
    request->z = z;

    if (!client_->service_is_ready()) {
      return false;
    }

    auto result_future = client_->async_send_request(request);
    if (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      return false;
    }

    auto response = result_future.get();
    if (!response->valid) {
      return false;  // outside map or map not ready
    }
    return response->distance > (drone_radius_ + safety_margin_);
  }

private:
  rclcpp::Client<esdf_msgs::srv::GetDistance>::SharedPtr client_;
  double drone_radius_;
  double safety_margin_;
};

class RRTStarPlannerNode : public rclcpp::Node
{
public:
  explicit RRTStarPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("rrt_star_planner", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("map_frame_id", "map");
    declare_parameter<std::string>("base_frame_id", "base_link");
    declare_parameter<double>("drone_radius", 0.25);
    declare_parameter<double>("safety_margin", 0.15);
    declare_parameter<double>("x_min", -10.0);
    declare_parameter<double>("x_max", 10.0);
    declare_parameter<double>("y_min", -10.0);
    declare_parameter<double>("y_max", 10.0);
    declare_parameter<double>("z_min", -0.5);
    declare_parameter<double>("z_max", 3.0);
    declare_parameter<double>("goal_bias", 0.15);
    declare_parameter<double>("max_planning_time", 0.5);
    declare_parameter<double>("replan_rate", 2.0);
    declare_parameter<int>("path_samples", 50);

    map_frame_id_ = get_parameter("map_frame_id").as_string();
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    drone_radius_ = get_parameter("drone_radius").as_double();
    safety_margin_ = get_parameter("safety_margin").as_double();
    x_min_ = get_parameter("x_min").as_double();
    x_max_ = get_parameter("x_max").as_double();
    y_min_ = get_parameter("y_min").as_double();
    y_max_ = get_parameter("y_max").as_double();
    z_min_ = get_parameter("z_min").as_double();
    z_max_ = get_parameter("z_max").as_double();
    goal_bias_ = get_parameter("goal_bias").as_double();
    max_planning_time_ = get_parameter("max_planning_time").as_double();
    replan_rate_ = get_parameter("replan_rate").as_double();
    path_samples_ = get_parameter("path_samples").as_int();

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10, std::bind(&RRTStarPlannerNode::goalCallback, this, std::placeholders::_1));
    path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
    tree_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("rrt_tree", 10);
    path_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("path_markers", 10);
    planning_active_pub_ = create_publisher<std_msgs::msg::Bool>("planning_active", 10);
    esdf_client_ = create_client<esdf_msgs::srv::GetDistance>("get_distance");

    replan_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / replan_rate_),
      std::bind(&RRTStarPlannerNode::replanTimerCallback, this));

    RCLCPP_INFO(get_logger(),
      "RRT* planner: map_frame=%s, drone_radius=%.2f, safety_margin=%.2f, replan=%.1f Hz",
      map_frame_id_.c_str(), drone_radius_, safety_margin_, replan_rate_);
  }

private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (msg->header.frame_id != "" && msg->header.frame_id != map_frame_id_) {
      RCLCPP_WARN(get_logger(), "Goal frame '%s' != map_frame '%s'; store anyway",
        msg->header.frame_id.c_str(), map_frame_id_.c_str());
    }
    std::lock_guard<std::mutex> lock(goal_mutex_);
    goal_pose_ = *msg;
    goal_pose_.header.frame_id = map_frame_id_;
    has_goal_ = true;
  }

  void replanTimerCallback()
  {
    if (!has_goal_) {
      return;
    }
    if (!esdf_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "RRT*: ESDF service not ready");
      return;
    }

    geometry_msgs::msg::PoseStamped start_pose;
    if (!getCurrentPose(start_pose)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "RRT*: no TF map->base_link");
      publishPlanningActive(false);
      return;
    }

    geometry_msgs::msg::PoseStamped goal;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      goal = goal_pose_;
    }

    runPlanning(start_pose, goal);
  }

  bool getCurrentPose(geometry_msgs::msg::PoseStamped & out)
  {
    try {
      geometry_msgs::msg::TransformStamped tf = tf_buffer_.lookupTransform(
        map_frame_id_, base_frame_id_, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.2));
      out.header = tf.header;
      out.pose.position.x = tf.transform.translation.x;
      out.pose.position.y = tf.transform.translation.y;
      out.pose.position.z = tf.transform.translation.z;
      out.pose.orientation = tf.transform.rotation;
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  static double yawFromQuaternion(double qx, double qy, double qz, double qw)
  {
    return std::atan2(
      2.0 * (qw * qz + qx * qy),
      1.0 - 2.0 * (qy * qy + qz * qz));
  }

  void runPlanning(
    const geometry_msgs::msg::PoseStamped & start_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose)
  {
    publishPlanningActive(true);

    auto space = std::make_shared<ob::CompoundStateSpace>();

    auto r3 = std::make_shared<ob::RealVectorStateSpace>(3);
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, x_min_);
    bounds.setHigh(0, x_max_);
    bounds.setLow(1, y_min_);
    bounds.setHigh(1, y_max_);
    bounds.setLow(2, z_min_);
    bounds.setHigh(2, z_max_);
    r3->setBounds(bounds);
    space->addSubspace(r3, 1.0);

    auto so2 = std::make_shared<ob::SO2StateSpace>();
    space->addSubspace(so2, 1.0);

    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    auto validity_checker = std::make_shared<EsdfStateValidityChecker>(
      esdf_client_, si, drone_radius_, safety_margin_);
    si->setStateValidityChecker(validity_checker);
    si->setStateValidityCheckingResolution(0.05);
    si->setup();

    ob::ScopedState<> start(space);
    start->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] =
      start_pose.pose.position.x;
    start->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1] =
      start_pose.pose.position.y;
    start->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[2] =
      start_pose.pose.position.z;
    start->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1)->value =
      yawFromQuaternion(
      start_pose.pose.orientation.x, start_pose.pose.orientation.y,
      start_pose.pose.orientation.z, start_pose.pose.orientation.w);

    ob::ScopedState<> goal(space);
    goal->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] =
      goal_pose.pose.position.x;
    goal->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1] =
      goal_pose.pose.position.y;
    goal->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[2] =
      goal_pose.pose.position.z;
    goal->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1)->value =
      yawFromQuaternion(
      goal_pose.pose.orientation.x, goal_pose.pose.orientation.y,
      goal_pose.pose.orientation.z, goal_pose.pose.orientation.w);

    ob::ProblemDefinitionPtr pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->addStartState(start);
    auto goal_ptr = std::make_shared<ob::GoalState>(si);
    goal_ptr->setState(goal);
    pdef->setGoal(goal_ptr);

    auto planner = std::make_shared<og::RRTstar>(si);
    planner->setProblemDefinition(pdef);
    planner->setup();
    planner->setGoalBias(goal_bias_);

    ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(max_planning_time_));

    publishPlanningActive(false);

    if (status != ob::PlannerStatus::EXACT_SOLUTION && status != ob::PlannerStatus::APPROXIMATE_SOLUTION) {
      RCLCPP_WARN(get_logger(), "RRT*: no solution (timeout or invalid)");
      publishEmptyPath();
      return;
    }

    og::PathGeometric * path = pdef->getSolutionPath()->as<og::PathGeometric>();
    if (!path || path->getStateCount() < 2) {
      publishEmptyPath();
      return;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = map_frame_id_;
    path_msg.header.stamp = now();

    std::vector<ob::State *> states = path->getStates();
    for (size_t i = 0; i < states.size(); ++i) {
      const auto * comp = states[i]->as<ob::CompoundState>();
      const auto * pos = comp->as<ob::RealVectorStateSpace::StateType>(0);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = pos->values[0];
      pose.pose.position.y = pos->values[1];
      pose.pose.position.z = pos->values[2];
      double yaw = comp->as<ob::SO2StateSpace::StateType>(1)->value;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
      path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);

    // Optional: publish tree and path as markers (simplified: just path line)
    publishPathMarkers(path_msg);
  }

  void publishPlanningActive(bool active)
  {
    std_msgs::msg::Bool msg;
    msg.data = active;
    planning_active_pub_->publish(msg);
  }

  void publishEmptyPath()
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = map_frame_id_;
    path_msg.header.stamp = now();
    path_pub_->publish(path_msg);
  }

  void publishPathMarkers(const nav_msgs::msg::Path & path_msg)
  {
    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker line;
    line.header = path_msg.header;
    line.ns = "path";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.05;
    line.color.a = 1.0;
    line.color.r = 0.0;
    line.color.g = 1.0;
    line.color.b = 0.0;
    for (const auto & p : path_msg.poses) {
      geometry_msgs::msg::Point pt;
      pt.x = p.pose.position.x;
      pt.y = p.pose.position.y;
      pt.z = p.pose.position.z;
      line.points.push_back(pt);
    }
    if (!line.points.empty()) {
      ma.markers.push_back(line);
    }
    path_marker_pub_->publish(ma);
  }

  std::string map_frame_id_;
  std::string base_frame_id_;
  double drone_radius_;
  double safety_margin_;
  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
  double goal_bias_;
  double max_planning_time_;
  double replan_rate_;
  int path_samples_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr planning_active_pub_;
  rclcpp::Client<esdf_msgs::srv::GetDistance>::SharedPtr esdf_client_;
  rclcpp::TimerBase::SharedPtr replan_timer_;

  std::mutex goal_mutex_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  bool has_goal_{false};
};

}  // namespace rrt_star_planner

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rrt_star_planner::RRTStarPlannerNode>();
  // Multi-threaded executor so service responses can be received while timer callback runs
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
