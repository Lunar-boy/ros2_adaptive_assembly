#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

class PreGraspPlanningBridge
{
public:
  explicit PreGraspPlanningBridge(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    input_topic_(declare_parameter<std::string>("input_topic", "/pre_grasp_pose")),
    success_topic_(declare_parameter<std::string>(
        "success_topic", "/pre_grasp_plan_success")),
    status_topic_(declare_parameter<std::string>(
        "status_topic", "/pre_grasp_planning_status")),
    duration_topic_(declare_parameter<std::string>(
        "duration_topic", "/pre_grasp_planning_duration_ms")),
    publish_diagnostics_(declare_parameter<bool>("publish_diagnostics", true)),
    planning_group_(declare_parameter<std::string>("planning_group", "panda_arm")),
    planning_time_sec_(declare_parameter<double>("planning_time_sec", 5.0)),
    position_tolerance_(declare_parameter<double>("position_tolerance", 0.01)),
    orientation_tolerance_(declare_parameter<double>("orientation_tolerance", 0.10)),
    min_replan_distance_(declare_parameter<double>("min_replan_distance", 0.03)),
    move_group_(node_, planning_group_)
  {
    move_group_.setPlanningTime(planning_time_sec_);
    move_group_.setGoalPositionTolerance(position_tolerance_);
    move_group_.setGoalOrientationTolerance(orientation_tolerance_);

    plan_success_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
      success_topic_, 10);
    planning_status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
      status_topic_, 10);
    planning_duration_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(
      duration_topic_, 10);
    pre_grasp_subscription_ =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        pre_grasp_pose_callback(*message);
      });

    RCLCPP_INFO(
      node_->get_logger(),
      "Pre-grasp planning bridge ready for group '%s' using input topic '%s'. "
      "success_topic='%s', status_topic='%s', duration_topic='%s', "
      "publish_diagnostics=%s. Planning only; execution is intentionally "
      "disabled in this PR.",
      planning_group_.c_str(), input_topic_.c_str(), success_topic_.c_str(),
      status_topic_.c_str(), duration_topic_.c_str(),
      publish_diagnostics_ ? "true" : "false");
  }

private:
  template<typename ParameterT>
  ParameterT declare_parameter(const std::string & name, const ParameterT & default_value)
  {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<ParameterT>(name, default_value);
    }
    return node_->get_parameter(name).get_value<ParameterT>();
  }

  void pre_grasp_pose_callback(const geometry_msgs::msg::PoseStamped & pose)
  {
    const auto & position = pose.pose.position;
    RCLCPP_INFO(
      node_->get_logger(),
      "Received pre-grasp pose: frame='%s', x=%.3f, y=%.3f, z=%.3f",
      pose.header.frame_id.c_str(), position.x, position.y, position.z);

    std::optional<double> distance_from_last_plan;
    if (last_planned_pose_.has_value()) {
      distance_from_last_plan = position_distance(pose, last_planned_pose_.value());
      if (distance_from_last_plan.value() < min_replan_distance_) {
        RCLCPP_INFO(
          node_->get_logger(),
          "Skipping MoveIt2 planning: target moved %.3f m, below min_replan_distance "
          "%.3f m. Publishing diagnostics with event=skipped_small_motion.",
          distance_from_last_plan.value(), min_replan_distance_);
        publish_plan_success(false);
        publish_planning_diagnostics(
          "skipped_small_motion", pose, distance_from_last_plan, 0.0);
        return;
      }
    }

    move_group_.setStartStateToCurrentState();
    if (!pose.header.frame_id.empty()) {
      move_group_.setPoseReferenceFrame(pose.header.frame_id);
    }
    move_group_.setPoseTarget(pose);

    RCLCPP_INFO(
      node_->get_logger(),
      "Requesting MoveIt2 plan to pre-grasp pose. Execution is intentionally "
      "disabled in this PR.");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planning_start_time = std::chrono::steady_clock::now();
    const bool planning_succeeded = static_cast<bool>(move_group_.plan(plan));
    const auto planning_end_time = std::chrono::steady_clock::now();
    const double duration_ms =
      std::chrono::duration<double, std::milli>(
      planning_end_time - planning_start_time).count();

    publish_plan_success(planning_succeeded);

    if (planning_succeeded) {
      publish_planning_diagnostics(
        "success", pose, distance_from_last_plan, duration_ms);
      RCLCPP_INFO(
        node_->get_logger(),
        "MoveIt2 planning succeeded in %.3f ms. Plan was not executed.",
        duration_ms);
    } else {
      publish_planning_diagnostics(
        "failure", pose, distance_from_last_plan, duration_ms);
      RCLCPP_WARN(
        node_->get_logger(),
        "MoveIt2 planning failed after %.3f ms. No trajectory execution was "
        "attempted.",
        duration_ms);
    }

    last_planned_pose_ = pose;
  }

  static double position_distance(
    const geometry_msgs::msg::PoseStamped & first,
    const geometry_msgs::msg::PoseStamped & second)
  {
    const double dx = first.pose.position.x - second.pose.position.x;
    const double dy = first.pose.position.y - second.pose.position.y;
    const double dz = first.pose.position.z - second.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  void publish_plan_success(const bool planning_succeeded)
  {
    std_msgs::msg::Bool message;
    message.data = planning_succeeded;
    plan_success_publisher_->publish(message);
  }

  void publish_planning_diagnostics(
    const std::string & event,
    const geometry_msgs::msg::PoseStamped & pose,
    const std::optional<double> & distance_from_last_plan,
    const double duration_ms)
  {
    if (!publish_diagnostics_) {
      return;
    }

    std_msgs::msg::String status_message;
    status_message.data = build_status_message(
      event, pose, distance_from_last_plan, duration_ms);
    planning_status_publisher_->publish(status_message);

    std_msgs::msg::Float64 duration_message;
    duration_message.data = duration_ms;
    planning_duration_publisher_->publish(duration_message);
  }

  std::string build_status_message(
    const std::string & event,
    const geometry_msgs::msg::PoseStamped & pose,
    const std::optional<double> & distance_from_last_plan,
    const double duration_ms) const
  {
    const auto & position = pose.pose.position;

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6);
    stream << "event=" << event;
    stream << ";frame=" << pose.header.frame_id;
    stream << ";x=" << position.x;
    stream << ";y=" << position.y;
    stream << ";z=" << position.z;
    stream << ";distance_from_last_plan=";
    if (distance_from_last_plan.has_value()) {
      stream << distance_from_last_plan.value();
    } else {
      stream << "none";
    }
    stream << ";min_replan_distance=" << min_replan_distance_;
    stream << ";duration_ms=" << duration_ms;
    stream << ";execution=false";
    return stream.str();
  }

  rclcpp::Node::SharedPtr node_;
  std::string input_topic_;
  std::string success_topic_;
  std::string status_topic_;
  std::string duration_topic_;
  bool publish_diagnostics_;
  std::string planning_group_;
  double planning_time_sec_;
  double position_tolerance_;
  double orientation_tolerance_;
  double min_replan_distance_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr plan_success_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planning_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr planning_duration_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pre_grasp_subscription_;
  std::optional<geometry_msgs::msg::PoseStamped> last_planned_pose_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "pre_grasp_planning_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto planning_bridge = std::make_shared<PreGraspPlanningBridge>(node);

  rclcpp::spin(node);
  planning_bridge.reset();
  rclcpp::shutdown();
  return 0;
}
