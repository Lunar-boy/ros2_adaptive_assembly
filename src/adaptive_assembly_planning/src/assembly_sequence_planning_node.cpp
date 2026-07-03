#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
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
    grasp_topic_(declare_parameter<std::string>(
        "grasp_topic", "/panda_grasp_pose")),
    assembly_topic_(declare_parameter<std::string>(
        "assembly_topic", "/panda_assembly_pose")),
    pre_place_topic_(declare_parameter<std::string>("pre_place_topic", "/panda_pre_place_pose")),
    place_topic_(declare_parameter<std::string>("place_topic", "/panda_place_pose")),
    retreat_topic_(declare_parameter<std::string>("retreat_topic", "/panda_retreat_pose")),
    success_topic_(declare_parameter<std::string>(
        "success_topic", "/assembly_sequence_plan_success")),
    status_topic_(declare_parameter<std::string>(
        "status_topic", "/assembly_sequence_planning_status")),
    duration_topic_(declare_parameter<std::string>(
        "duration_topic", "/assembly_sequence_planning_duration_ms")),
    stage_status_topic_(declare_parameter<std::string>(
        "stage_status_topic", "/assembly_sequence_stage_status")),
    stage_success_topic_(declare_parameter<std::string>(
        "stage_success_topic", "/assembly_sequence_stage_success")),
    stage_duration_topic_(declare_parameter<std::string>(
        "stage_duration_topic", "/assembly_sequence_stage_duration_ms")),
    pre_grasp_trajectory_topic_(declare_parameter<std::string>(
        "pre_grasp_trajectory_topic", "/pre_grasp_trajectory")),
    grasp_trajectory_topic_(declare_parameter<std::string>(
        "grasp_trajectory_topic", "/grasp_trajectory")),
    assembly_trajectory_topic_(declare_parameter<std::string>(
        "assembly_trajectory_topic", "/assembly_trajectory")),
    pre_place_trajectory_topic_(declare_parameter<std::string>("pre_place_trajectory_topic", "/pre_place_trajectory")),
    place_trajectory_topic_(declare_parameter<std::string>("place_trajectory_topic", "/place_trajectory")),
    retreat_trajectory_topic_(declare_parameter<std::string>("retreat_trajectory_topic", "/retreat_trajectory")),
    trajectory_status_topic_(declare_parameter<std::string>(
        "trajectory_status_topic", "/assembly_sequence_trajectory_status")),
    publish_diagnostics_(declare_parameter<bool>("publish_diagnostics", true)),
    publish_trajectories_(declare_parameter<bool>("publish_trajectories", true)),
    require_grasp_pose_(declare_parameter<bool>("require_grasp_pose", false)),
    require_place_sequence_(declare_parameter<bool>("require_place_sequence", false)),
    planning_group_(declare_parameter<std::string>("planning_group", "panda_arm")),
    planner_id_(declare_parameter<std::string>("planner_id", "")),
    num_planning_attempts_(declare_parameter<int>("num_planning_attempts", 1)),
    planning_time_sec_(declare_parameter<double>("planning_time_sec", 5.0)),
    position_tolerance_(declare_parameter<double>("position_tolerance", 0.01)),
    orientation_tolerance_(declare_parameter<double>("orientation_tolerance", 0.10)),
    start_state_mode_(declare_parameter<std::string>("start_state_mode", "current")),
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
    stage_status_publisher_ =
      node_->create_publisher<std_msgs::msg::String>(stage_status_topic_, 10);
    stage_success_publisher_ =
      node_->create_publisher<std_msgs::msg::Bool>(stage_success_topic_, 10);
    stage_duration_publisher_ =
      node_->create_publisher<std_msgs::msg::Float64>(stage_duration_topic_, 10);
    pre_grasp_trajectory_publisher_ =
      node_->create_publisher<moveit_msgs::msg::RobotTrajectory>(
      pre_grasp_trajectory_topic_, 10);
    grasp_trajectory_publisher_ =
      node_->create_publisher<moveit_msgs::msg::RobotTrajectory>(
      grasp_trajectory_topic_, 10);
    assembly_trajectory_publisher_ =
      node_->create_publisher<moveit_msgs::msg::RobotTrajectory>(
      assembly_trajectory_topic_, 10);
    pre_place_trajectory_publisher_ = node_->create_publisher<moveit_msgs::msg::RobotTrajectory>(pre_place_trajectory_topic_, 10);
    place_trajectory_publisher_ = node_->create_publisher<moveit_msgs::msg::RobotTrajectory>(place_trajectory_topic_, 10);
    retreat_trajectory_publisher_ = node_->create_publisher<moveit_msgs::msg::RobotTrajectory>(retreat_trajectory_topic_, 10);
    trajectory_status_publisher_ =
      node_->create_publisher<std_msgs::msg::String>(trajectory_status_topic_, 10);

    pre_grasp_subscription_ =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      pre_grasp_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        pre_grasp_pose_ = *message;
        pre_grasp_updated_ = true;
        try_plan_sequence();
      });
    grasp_subscription_ =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      grasp_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        grasp_pose_ = *message;
        grasp_updated_ = true;
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
    pre_place_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(pre_place_topic_, 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {pre_place_pose_ = *message; pre_place_updated_ = true; try_plan_sequence();});
    place_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(place_topic_, 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {place_pose_ = *message; place_updated_ = true; try_plan_sequence();});
    retreat_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(retreat_topic_, 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {retreat_pose_ = *message; retreat_updated_ = true; try_plan_sequence();});

    RCLCPP_INFO(
      node_->get_logger(),
      "Assembly sequence planner ready: pre_grasp_topic='%s', grasp_topic='%s', "
      "assembly_topic='%s', require_grasp_pose=%s, "
      "planning_group='%s', planner_id='%s', num_planning_attempts=%d, "
      "planning_time_sec=%.3f, position_tolerance=%.3f, "
      "orientation_tolerance=%.3f, start_state_mode='%s', publish_diagnostics=%s. "
      "Stage diagnostics: status='%s', success='%s', duration='%s'. "
      "Trajectory export: enabled=%s, pre_grasp='%s', grasp='%s', assembly='%s', status='%s'. "
      "All sequence stages are planned only; execution is disabled.",
      pre_grasp_topic_.c_str(), grasp_topic_.c_str(), assembly_topic_.c_str(),
      require_grasp_pose_ ? "true" : "false", planning_group_.c_str(),
      planner_id_.c_str(), num_planning_attempts_, planning_time_sec_,
      position_tolerance_, orientation_tolerance_, start_state_mode_.c_str(),
      publish_diagnostics_ ? "true" : "false", stage_status_topic_.c_str(),
      stage_success_topic_.c_str(), stage_duration_topic_.c_str(),
      publish_trajectories_ ? "true" : "false", pre_grasp_trajectory_topic_.c_str(),
      grasp_trajectory_topic_.c_str(), assembly_trajectory_topic_.c_str(),
      trajectory_status_topic_.c_str());
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
    if (start_state_mode_ != "current" && start_state_mode_ != "fixed") {
      RCLCPP_WARN(
        node_->get_logger(),
        "start_state_mode='%s' is invalid; using 'current'.",
        start_state_mode_.c_str());
      start_state_mode_ = "current";
    }
  }

  void try_plan_sequence()
  {
    if (!pre_grasp_pose_.has_value() || (!require_place_sequence_ && !assembly_pose_.has_value()) ||
      !pre_grasp_updated_ || (!require_place_sequence_ && !assembly_updated_) ||
      (require_grasp_pose_ && (!grasp_pose_.has_value() || !grasp_updated_)))
    {
      return;
    }
    if (require_place_sequence_ &&
      (!grasp_pose_.has_value() || !grasp_updated_ || !pre_place_pose_.has_value() ||
      !pre_place_updated_ || !place_pose_.has_value() || !place_updated_ ||
      !retreat_pose_.has_value() || !retreat_updated_)) {
      return;
    }

    pre_grasp_updated_ = false;
    grasp_updated_ = false;
    assembly_updated_ = false;
    pre_place_updated_ = false;
    place_updated_ = false;
    retreat_updated_ = false;
    if (require_place_sequence_) {
      plan_place_sequence();
      return;
    }
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

    set_pre_grasp_start_state();
    set_pose_target(pre_grasp_pose);
    moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_plan;
    const auto pre_grasp_result = plan_stage(pre_grasp_plan);
    total_duration_ms += pre_grasp_result.duration_ms;
    move_group_.clearPoseTargets();
    publish_stage_result(
      "pre_grasp", pre_grasp_result.succeeded, pre_grasp_result.duration_ms);

    if (!pre_grasp_result.succeeded) {
      publish_trajectory_status(
        "skipped", "pre_grasp", pre_grasp_trajectory_topic_, nullptr, "planning_failed");
      if (require_grasp_pose_) {
        publish_trajectory_status(
          "skipped", "grasp", grasp_trajectory_topic_, nullptr, "pre_grasp_failed");
      }
      publish_trajectory_status(
        "skipped", "assembly", assembly_trajectory_topic_, nullptr, "pre_grasp_failed");
      publish_result(false, "failure", "pre_grasp", planned_stage_count, total_duration_ms);
      RCLCPP_WARN(
        node_->get_logger(),
        "Assembly sequence planning failed at pre_grasp after %.3f ms. No execution attempted.",
        total_duration_ms);
      return;
    }
    ++planned_stage_count;
    publish_trajectory(
      "pre_grasp", pre_grasp_plan.trajectory, pre_grasp_trajectory_topic_,
      pre_grasp_trajectory_publisher_);

    auto assembly_start_state = final_state_from_plan(pre_grasp_plan);
    if (!assembly_start_state.has_value()) {
      if (require_grasp_pose_) {
        publish_trajectory_status(
          "skipped", "grasp", grasp_trajectory_topic_, nullptr,
          "invalid_pre_grasp_trajectory");
      }
      publish_trajectory_status(
        "skipped", "assembly", assembly_trajectory_topic_, nullptr,
        "invalid_pre_grasp_trajectory");
      publish_result(false, "failure", "assembly", planned_stage_count, total_duration_ms);
      RCLCPP_WARN(
        node_->get_logger(),
        "Pre-grasp plan had no final joint state; assembly stage was not planned. "
        "No execution attempted.");
      return;
    }

    if (require_grasp_pose_) {
      move_group_.setStartState(assembly_start_state.value());
      set_pose_target(grasp_pose_.value());
      moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
      const auto grasp_result = plan_stage(grasp_plan);
      total_duration_ms += grasp_result.duration_ms;
      move_group_.clearPoseTargets();
      publish_stage_result("grasp", grasp_result.succeeded, grasp_result.duration_ms);

      if (!grasp_result.succeeded) {
        publish_trajectory_status(
          "skipped", "grasp", grasp_trajectory_topic_, nullptr, "planning_failed");
        publish_trajectory_status(
          "skipped", "assembly", assembly_trajectory_topic_, nullptr, "grasp_failed");
        publish_result(false, "failure", "grasp", planned_stage_count, total_duration_ms);
        RCLCPP_WARN(node_->get_logger(), "Assembly sequence planning failed at grasp.");
        return;
      }
      ++planned_stage_count;
      publish_trajectory(
        "grasp", grasp_plan.trajectory, grasp_trajectory_topic_,
        grasp_trajectory_publisher_);
      assembly_start_state = final_state_from_plan(grasp_plan);
      if (!assembly_start_state.has_value()) {
        publish_trajectory_status(
          "skipped", "assembly", assembly_trajectory_topic_, nullptr,
          "invalid_grasp_trajectory");
        publish_result(false, "failure", "assembly", planned_stage_count, total_duration_ms);
        return;
      }
    }

    move_group_.setStartState(assembly_start_state.value());
    set_pose_target(assembly_pose);
    moveit::planning_interface::MoveGroupInterface::Plan assembly_plan;
    const auto assembly_result = plan_stage(assembly_plan);
    total_duration_ms += assembly_result.duration_ms;
    move_group_.clearPoseTargets();
    publish_stage_result(
      "assembly", assembly_result.succeeded, assembly_result.duration_ms);

    if (!assembly_result.succeeded) {
      publish_trajectory_status(
        "skipped", "assembly", assembly_trajectory_topic_, nullptr, "planning_failed");
      publish_result(false, "failure", "assembly", planned_stage_count, total_duration_ms);
      RCLCPP_WARN(
        node_->get_logger(),
        "Assembly sequence planning failed at assembly after %.3f ms total. "
        "No execution attempted.", total_duration_ms);
      return;
    }

    ++planned_stage_count;
    publish_trajectory(
      "assembly", assembly_plan.trajectory, assembly_trajectory_topic_,
      assembly_trajectory_publisher_);
    publish_result(true, "success", "none", planned_stage_count, total_duration_ms);
    RCLCPP_INFO(
      node_->get_logger(),
      "Assembly sequence planning succeeded for %zu stages in %.3f ms total. "
      "No trajectory was executed by the planner.", planned_stage_count, total_duration_ms);
  }

  void plan_place_sequence()
  {
    const std::array<std::string, 5> names = {"pre_grasp", "grasp", "pre_place", "place", "retreat"};
    const std::array<geometry_msgs::msg::PoseStamped, 5> poses = {
      pre_grasp_pose_.value(), grasp_pose_.value(), pre_place_pose_.value(),
      place_pose_.value(), retreat_pose_.value()};
    const std::array<std::string, 5> topics = {
      pre_grasp_trajectory_topic_, grasp_trajectory_topic_, pre_place_trajectory_topic_,
      place_trajectory_topic_, retreat_trajectory_topic_};
    const std::array<rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr, 5> publishers = {
      pre_grasp_trajectory_publisher_, grasp_trajectory_publisher_, pre_place_trajectory_publisher_,
      place_trajectory_publisher_, retreat_trajectory_publisher_};
    double total_duration_ms = 0.0;
    std::size_t planned_stage_count = 0;
    std::optional<moveit_msgs::msg::RobotState> start_state;
    set_pre_grasp_start_state();
    for (std::size_t index = 0; index < names.size(); ++index) {
      if (index > 0) {
        if (!start_state.has_value()) {
          for (std::size_t remaining = index; remaining < names.size(); ++remaining) {
            publish_trajectory_status("skipped", names[remaining], topics[remaining], nullptr, "invalid_previous_trajectory");
          }
          publish_result(false, "failure", names[index], planned_stage_count, total_duration_ms);
          return;
        }
        move_group_.setStartState(start_state.value());
      }
      set_pose_target(poses[index]);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto result = plan_stage(plan);
      total_duration_ms += result.duration_ms;
      move_group_.clearPoseTargets();
      publish_stage_result(names[index], result.succeeded, result.duration_ms);
      if (!result.succeeded) {
        publish_trajectory_status("skipped", names[index], topics[index], nullptr, "planning_failed");
        for (std::size_t remaining = index + 1; remaining < names.size(); ++remaining) {
          publish_trajectory_status("skipped", names[remaining], topics[remaining], nullptr, names[index] + "_failed");
        }
        publish_result(false, "failure", names[index], planned_stage_count, total_duration_ms);
        return;
      }
      ++planned_stage_count;
      publish_trajectory(names[index], plan.trajectory, topics[index], publishers[index]);
      start_state = final_state_from_plan(plan);
    }
    publish_result(true, "success", "none", planned_stage_count, total_duration_ms);
    RCLCPP_INFO(node_->get_logger(), "Five-stage place sequence planning succeeded; execution remains disabled.");
  }

  void set_pose_target(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!pose.header.frame_id.empty()) {
      move_group_.setPoseReferenceFrame(pose.header.frame_id);
    }
    move_group_.setPoseTarget(pose);
  }

  void set_pre_grasp_start_state()
  {
    if (start_state_mode_ == "current") {
      move_group_.setStartStateToCurrentState();
      return;
    }

    constexpr std::array<const char *, 7> joint_names = {
      "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
      "panda_joint5", "panda_joint6", "panda_joint7"
    };
    constexpr std::array<double, 7> joint_positions = {
      0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785
    };

    moveit_msgs::msg::RobotState fixed_state;
    fixed_state.joint_state.name.assign(joint_names.begin(), joint_names.end());
    fixed_state.joint_state.position.assign(
      joint_positions.begin(), joint_positions.end());
    fixed_state.is_diff = false;
    move_group_.setStartState(fixed_state);

    RCLCPP_INFO(
      node_->get_logger(),
      "Using deterministic fixed Panda start state for pre-grasp planning. "
      "Execution remains disabled.");
  }

  struct StagePlanResult
  {
    bool succeeded;
    double duration_ms;
  };

  StagePlanResult plan_stage(
    moveit::planning_interface::MoveGroupInterface::Plan & plan)
  {
    const auto start = std::chrono::steady_clock::now();
    const bool succeeded = static_cast<bool>(move_group_.plan(plan));
    const auto end = std::chrono::steady_clock::now();
    const double duration_ms =
      std::chrono::duration<double, std::milli>(end - start).count();
    return {succeeded, duration_ms};
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

  void publish_trajectory(
    const std::string & stage,
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const std::string & topic,
    const rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr & publisher)
  {
    if (!publish_trajectories_) {
      publish_trajectory_status(
        "skipped", stage, topic, &trajectory, "publishing_disabled");
      return;
    }

    publisher->publish(trajectory);
    publish_trajectory_status("published", stage, topic, &trajectory, "");
  }

  void publish_trajectory_status(
    const std::string & event,
    const std::string & stage,
    const std::string & topic,
    const moveit_msgs::msg::RobotTrajectory * trajectory,
    const std::string & reason)
  {
    std_msgs::msg::String status_message;
    std::ostringstream status;
    status << "event=" << event
           << ";stage=" << stage;
    if (trajectory != nullptr) {
      status << ";point_count=" << trajectory->joint_trajectory.points.size()
             << ";joint_count=" << trajectory->joint_trajectory.joint_names.size();
    }
    status << ";topic=" << topic;
    if (!reason.empty()) {
      status << ";reason=" << reason;
    }
    status << ";execution=false";
    status_message.data = status.str();
    trajectory_status_publisher_->publish(status_message);
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
           << ";start_state_mode=" << start_state_mode_
           << ";execution=false";
    status_message.data = status.str();
    status_publisher_->publish(status_message);

    std_msgs::msg::Float64 duration_message;
    duration_message.data = total_duration_ms;
    duration_publisher_->publish(duration_message);
  }

  void publish_stage_result(
    const std::string & stage,
    const bool success,
    const double duration_ms)
  {
    std_msgs::msg::Bool success_message;
    success_message.data = success;
    stage_success_publisher_->publish(success_message);

    if (!publish_diagnostics_) {
      return;
    }

    std_msgs::msg::String status_message;
    std::ostringstream status;
    status << std::fixed << std::setprecision(6)
           << "event=" << (success ? "success" : "failure")
           << ";stage=" << stage
           << ";duration_ms=" << duration_ms
           << ";planning_group=" << planning_group_
           << ";planner_id=" << (planner_id_.empty() ? "<default>" : planner_id_)
           << ";num_planning_attempts=" << num_planning_attempts_
           << ";planning_time_sec=" << planning_time_sec_
           << ";position_tolerance=" << position_tolerance_
           << ";orientation_tolerance=" << orientation_tolerance_
           << ";start_state_mode=" << start_state_mode_
           << ";execution=false";
    status_message.data = status.str();
    stage_status_publisher_->publish(status_message);

    std_msgs::msg::Float64 duration_message;
    duration_message.data = duration_ms;
    stage_duration_publisher_->publish(duration_message);
  }

  rclcpp::Node::SharedPtr node_;
  std::string pre_grasp_topic_;
  std::string grasp_topic_;
  std::string assembly_topic_;
  std::string pre_place_topic_, place_topic_, retreat_topic_;
  std::string success_topic_;
  std::string status_topic_;
  std::string duration_topic_;
  std::string stage_status_topic_;
  std::string stage_success_topic_;
  std::string stage_duration_topic_;
  std::string pre_grasp_trajectory_topic_;
  std::string grasp_trajectory_topic_;
  std::string assembly_trajectory_topic_;
  std::string pre_place_trajectory_topic_, place_trajectory_topic_, retreat_trajectory_topic_;
  std::string trajectory_status_topic_;
  bool publish_diagnostics_;
  bool publish_trajectories_;
  bool require_grasp_pose_;
  bool require_place_sequence_;
  std::string planning_group_;
  std::string planner_id_;
  int num_planning_attempts_;
  double planning_time_sec_;
  double position_tolerance_;
  double orientation_tolerance_;
  std::string start_state_mode_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr success_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr duration_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stage_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stage_success_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stage_duration_publisher_;
  rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr
    pre_grasp_trajectory_publisher_;
  rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr
    grasp_trajectory_publisher_;
  rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr
    assembly_trajectory_publisher_;
  rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr pre_place_trajectory_publisher_, place_trajectory_publisher_, retreat_trajectory_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trajectory_status_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pre_grasp_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr assembly_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pre_place_subscription_, place_subscription_, retreat_subscription_;
  std::optional<geometry_msgs::msg::PoseStamped> pre_grasp_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> grasp_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> assembly_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> pre_place_pose_, place_pose_, retreat_pose_;
  bool pre_grasp_updated_{false};
  bool grasp_updated_{false};
  bool assembly_updated_{false};
  bool pre_place_updated_{false}, place_updated_{false}, retreat_updated_{false};
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
