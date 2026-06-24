#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class PreGraspPlanningBridge
{
public:
  explicit PreGraspPlanningBridge(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    input_topic_(declare_parameter<std::string>("input_topic", "/pre_grasp_pose")),
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
      "/pre_grasp_plan_success", 10);
    pre_grasp_subscription_ =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        pre_grasp_pose_callback(*message);
      });

    RCLCPP_INFO(
      node_->get_logger(),
      "Pre-grasp planning bridge ready for group '%s' using input topic '%s'. "
      "Planning only; execution is intentionally disabled in this PR.",
      planning_group_.c_str(), input_topic_.c_str());
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

    if (last_planned_pose_.has_value()) {
      const double distance = position_distance(pose, last_planned_pose_.value());
      if (distance < min_replan_distance_) {
        RCLCPP_INFO(
          node_->get_logger(),
          "Skipping MoveIt2 planning: target moved %.3f m, below min_replan_distance "
          "%.3f m.",
          distance, min_replan_distance_);
        publish_plan_success(false);
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
    const bool planning_succeeded = static_cast<bool>(move_group_.plan(plan));
    publish_plan_success(planning_succeeded);

    if (planning_succeeded) {
      RCLCPP_INFO(
        node_->get_logger(),
        "MoveIt2 planning succeeded. Plan was not executed.");
    } else {
      RCLCPP_WARN(
        node_->get_logger(),
        "MoveIt2 planning failed. No trajectory execution was attempted.");
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

  rclcpp::Node::SharedPtr node_;
  std::string input_topic_;
  std::string planning_group_;
  double planning_time_sec_;
  double position_tolerance_;
  double orientation_tolerance_;
  double min_replan_distance_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr plan_success_publisher_;
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
