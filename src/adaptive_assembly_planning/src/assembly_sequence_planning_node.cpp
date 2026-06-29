#include <algorithm>
#include <chrono>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

class AssemblySequencePlanningNode
{
public:
  explicit AssemblySequencePlanningNode(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    pre_grasp_topic_(declare_parameter<std::string>(
        "pre_grasp_topic", "/panda_pre_grasp_pose")),
    assembly_topic_(declare_parameter<std::string>(
        "assembly_topic", "/panda_assembly_pose")),
    success_topic_(declare_parameter<std::string>(
        "success_topic", "/assembly_sequence_plan_success")),
    status_topic_(declare_parameter<std::string>(
        "status_topic", "/assembly_sequence_planning_status")),
    duration_topic_(declare_parameter<std::string>(
        "duration_topic", "/assembly_sequence_planning_duration_ms")),
    publish_diagnostics_(declare_parameter<bool>("publish_diagnostics", true)),
    planning_group_(declare_parameter<std::string>("planning_group", "panda_arm")),
    planner_id_(declare_parameter<std::string>("planner_id", "")),
    num_planning_attempts_(declare_parameter<int>("num_planning_attempts", 1)),
    planning_time_sec_(declare_parameter<double>("planning_time_sec", 5.0)),
    position_tolerance_(declare_parameter<double>("position_tolerance", 0.01)),
    orientation_tolerance_(declare_parameter<double>("orientation_tolerance", 0.10)),
    move_group_(node_, planning_group_)
  {
    validate_parameters();
    move_group_.setPlanningTime(planning_time_sec_);
    move_group_.setGoalPositionTolerance(position_tolerance_);
    move_group_.setGoalOrientationTolerance(orientation_tolerance_);
    move_group_.setNumPlanningAttempts(num_planning_attempts_);
    if (!planner_id_.empty()) {
      move_group_.setPlannerId(planner_id_);
    }

    success_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(success_topic_, 10);
    status_publisher_ = node_->create_publisher<std_msgs::msg::String>(status_topic_, 10);
    duration_publisher_ =
      node_->create_publisher<std_msgs::msg::Float64>(duration_topic_, 10);

    pre_grasp_subscription_ =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      pre_grasp_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        pre_grasp_pose_ = *message;
        pre_grasp_updated_ = true;
        try_plan_sequence();
      });
    assembly_subscription_ =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      assembly_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        assembly_pose_ = *message;
        assembly_updated_ = true;
        try_plan_sequence();
      });

    RCLCPP_INFO(
      node_->get_logger(),
      "Assembly sequence planner ready: pre_grasp_topic='%s', assembly_topic='%s', "
      "planning_group='%s', planner_id='%s', num_planning_attempts=%d, "
      "planning_time_sec=%.3f, position_tolerance=%.3f, "
      "orientation_tolerance=%.3f, publish_diagnostics=%s. "
      "The pre-grasp and assembly stages are planned only; execution is disabled.",
      pre_grasp_topic_.c_str(), assembly_topic_.c_str(), planning_group_.c_str(),
      planner_id_.c_str(), num_planning_attempts_, planning_time_sec_,
      position_tolerance_, orientation_tolerance_,
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

  void validate_parameters()
  {
    if (num_planning_attempts_ < 1) {
      RCLCPP_WARN(
        node_->get_logger(),
        "num_planning_attempts=%d is invalid; using 1.", num_planning_attempts_);
      num_planning_attempts_ = 1;
    }
    if (planning_time_sec_ <= 0.0) {
      RCLCPP_WARN(node_->get_logger(), "planning_time_sec must be positive; using 5.0.");
      planning_time_sec_ = 5.0;
    }
    if (position_tolerance_ <= 0.0) {
      RCLCPP_WARN(node_->get_logger(), "position_tolerance must be positive; using 0.01.");
      position_tolerance_ = 0.01;
    }
    if (orientation_tolerance_ <= 0.0) {
      RCLCPP_WARN(node_->get_logger(), "orientation_tolerance must be positive; using 0.10.");
      orientation_tolerance_ = 0.10;
    }
  }

  void try_plan_sequence()
  {
    if (!pre_grasp_pose_.has_value() || !assembly_pose_.has_value() ||
      !pre_grasp_updated_ || !assembly_updated_)
    {
      return;
    }

    pre_grasp_updated_ = false;
    assembly_updated_ = false;
    const auto & pre_grasp_pose = pre_grasp_pose_.value();
    const auto & assembly_pose = assembly_pose_.value();

    RCLCPP_INFO(
      node_->get_logger(),
      "Planning assembly sequence: pre_grasp frame='%s' (%.3f, %.3f, %.3f), "
      "assembly frame='%s' (%.3f, %.3f, %.3f). Execution is disabled.",
      pre_grasp_pose.header.frame_id.c_str(),
      pre_grasp_pose.pose.position.x, pre_grasp_pose.pose.position.y,
      pre_grasp_pose.pose.position.z, assembly_pose.header.frame_id.c_str(),
      assembly_pose.pose.position.x, assembly_pose.pose.position.y,
      assembly_pose.pose.position.z);

    double total_duration_ms = 0.0;
    std::size_t planned_stage_count = 0;

    move_group_.setStartStateToCurrentState();
    set_pose_target(pre_grasp_pose);
    moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_plan;
    const bool pre_grasp_succeeded = plan_stage(pre_grasp_plan, total_duration_ms);
    move_group_.clearPoseTargets();

    if (!pre_grasp_succeeded) {
      publish_result(false, "failure", "pre_grasp", planned_stage_count, total_duration_ms);
      RCLCPP_WARN(
        node_->get_logger(),
        "Assembly sequence planning failed at pre_grasp after %.3f ms. No execution attempted.",
        total_duration_ms);
      return;
    }
    ++planned_stage_count;

    const auto assembly_start_state = final_state_from_plan(pre_grasp_plan);
    if (!assembly_start_state.has_value()) {
      publish_result(false, "failure", "assembly", planned_stage_count, total_duration_ms);
      RCLCPP_WARN(
        node_->get_logger(),
        "Pre-grasp plan had no final joint state; assembly stage was not planned. "
        "No execution attempted.");
      return;
    }

    move_group_.setStartState(assembly_start_state.value());
    set_pose_target(assembly_pose);
    moveit::planning_interface::MoveGroupInterface::Plan assembly_plan;
    const bool assembly_succeeded = plan_stage(assembly_plan, total_duration_ms);
    move_group_.clearPoseTargets();

    if (!assembly_succeeded) {
      publish_result(false, "failure", "assembly", planned_stage_count, total_duration_ms);
      RCLCPP_WARN(
        node_->get_logger(),
        "Assembly sequence planning failed at assembly after %.3f ms total. "
        "No execution attempted.", total_duration_ms);
      return;
    }

    ++planned_stage_count;
    publish_result(true, "success", "none", planned_stage_count, total_duration_ms);
    RCLCPP_INFO(
      node_->get_logger(),
      "Assembly sequence planning succeeded for %zu stages in %.3f ms total. "
      "Neither trajectory was executed.", planned_stage_count, total_duration_ms);
  }

  void set_pose_target(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!pose.header.frame_id.empty()) {
      move_group_.setPoseReferenceFrame(pose.header.frame_id);
    }
    move_group_.setPoseTarget(pose);
  }

  bool plan_stage(
    moveit::planning_interface::MoveGroupInterface::Plan & plan,
    double & total_duration_ms)
  {
    const auto start = std::chrono::steady_clock::now();
    const bool succeeded = static_cast<bool>(move_group_.plan(plan));
    const auto end = std::chrono::steady_clock::now();
    total_duration_ms +=
      std::chrono::duration<double, std::milli>(end - start).count();
    return succeeded;
  }

  static std::optional<moveit_msgs::msg::RobotState> final_state_from_plan(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan)
  {
    const auto & joint_trajectory = plan.trajectory.joint_trajectory;
    if (joint_trajectory.points.empty()) {
      return std::nullopt;
    }

    moveit_msgs::msg::RobotState state = plan.start_state;
    const auto & final_positions = joint_trajectory.points.back().positions;
    const std::size_t count = std::min(
      joint_trajectory.joint_names.size(), final_positions.size());

    for (std::size_t index = 0; index < count; ++index) {
      const auto & joint_name = joint_trajectory.joint_names[index];
      const auto existing = std::find(
        state.joint_state.name.begin(), state.joint_state.name.end(), joint_name);
      if (existing == state.joint_state.name.end()) {
        state.joint_state.name.push_back(joint_name);
        state.joint_state.position.push_back(final_positions[index]);
      } else {
        const auto state_index = static_cast<std::size_t>(
          std::distance(state.joint_state.name.begin(), existing));
        if (state.joint_state.position.size() <= state_index) {
          state.joint_state.position.resize(state_index + 1, 0.0);
        }
        state.joint_state.position[state_index] = final_positions[index];
      }
    }
    return state;
  }

  void publish_result(
    const bool success,
    const std::string & event,
    const std::string & failed_stage,
    const std::size_t planned_stage_count,
    const double total_duration_ms)
  {
    std_msgs::msg::Bool success_message;
    success_message.data = success;
    success_publisher_->publish(success_message);

    if (!publish_diagnostics_) {
      return;
    }

    std_msgs::msg::String status_message;
    std::ostringstream status;
    status << std::fixed << std::setprecision(6)
           << "event=" << event
           << ";failed_stage=" << failed_stage
           << ";planned_stage_count=" << planned_stage_count
           << ";total_duration_ms=" << total_duration_ms
           << ";execution=false";
    status_message.data = status.str();
    status_publisher_->publish(status_message);

    std_msgs::msg::Float64 duration_message;
    duration_message.data = total_duration_ms;
    duration_publisher_->publish(duration_message);
  }

  rclcpp::Node::SharedPtr node_;
  std::string pre_grasp_topic_;
  std::string assembly_topic_;
  std::string success_topic_;
  std::string status_topic_;
  std::string duration_topic_;
  bool publish_diagnostics_;
  std::string planning_group_;
  std::string planner_id_;
  int num_planning_attempts_;
  double planning_time_sec_;
  double position_tolerance_;
  double orientation_tolerance_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr success_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr duration_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pre_grasp_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr assembly_subscription_;
  std::optional<geometry_msgs::msg::PoseStamped> pre_grasp_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> assembly_pose_;
  bool pre_grasp_updated_{false};
  bool assembly_updated_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "assembly_sequence_planning_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto planner = std::make_shared<AssemblySequencePlanningNode>(node);

  rclcpp::spin(node);
  planner.reset();
  rclcpp::shutdown();
  return 0;
}
