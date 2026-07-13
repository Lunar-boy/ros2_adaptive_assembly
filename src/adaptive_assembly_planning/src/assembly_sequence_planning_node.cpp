#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

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
    trajectory_status_topic_(declare_parameter<std::string>(
        "trajectory_status_topic", "/assembly_sequence_trajectory_status")),
    publish_diagnostics_(declare_parameter<bool>("publish_diagnostics", true)),
    publish_trajectories_(declare_parameter<bool>("publish_trajectories", true)),
    require_grasp_pose_(declare_parameter<bool>("require_grasp_pose", false)),
    require_place_sequence_(declare_parameter<bool>("require_place_sequence", false)),
    planning_group_(declare_parameter<std::string>("planning_group", "panda_arm")),
    end_effector_link_(declare_parameter<std::string>("end_effector_link", "panda_link8")),
    planner_id_(declare_parameter<std::string>("planner_id", "")),
    num_planning_attempts_(declare_parameter<int>("num_planning_attempts", 1)),
    planning_time_sec_(declare_parameter<double>("planning_time_sec", 5.0)),
    position_tolerance_(declare_parameter<double>("position_tolerance", 0.01)),
    orientation_tolerance_(declare_parameter<double>("orientation_tolerance", 0.10)),
    start_state_mode_(declare_parameter<std::string>("start_state_mode", "current")),
    move_group_(node_, planning_group_)
  {
    validate_parameters();
    configure_move_group();
    create_diagnostic_publishers();
    resolve_stage_names();
    create_stages();

    RCLCPP_INFO(
      node_->get_logger(),
      "Assembly sequence planner ready: stage_sequence='%s', planning_group='%s', "
      "end_effector_link='%s', planner_id='%s', num_planning_attempts=%d, planning_time_sec=%.3f, "
      "position_tolerance=%.3f, orientation_tolerance=%.3f, start_state_mode='%s', "
      "publish_diagnostics=%s, publish_trajectories=%s. All stages are plan-only; "
      "execution is disabled.",
      stage_sequence_.c_str(), planning_group_.c_str(), end_effector_link_.c_str(),
      planner_id_.c_str(),
      num_planning_attempts_, planning_time_sec_, position_tolerance_,
      orientation_tolerance_, start_state_mode_.c_str(),
      publish_diagnostics_ ? "true" : "false",
      publish_trajectories_ ? "true" : "false");
  }

private:
  struct StageConfig
  {
    std::string name;
    std::string pose_topic;
    std::string trajectory_topic;
    std::optional<geometry_msgs::msg::PoseStamped> pose;
    bool updated{false};
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription;
    rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr trajectory_publisher;
  };

  struct StagePlanResult
  {
    bool succeeded;
    double duration_ms;
  };

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
      RCLCPP_WARN(node_->get_logger(), "num_planning_attempts is invalid; using 1.");
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
        node_->get_logger(), "start_state_mode='%s' is invalid; using 'current'.",
        start_state_mode_.c_str());
      start_state_mode_ = "current";
    }
  }

  void configure_move_group()
  {
    const auto robot_model = move_group_.getRobotModel();
    if (!robot_model || !robot_model->hasLinkModel(end_effector_link_)) {
      throw std::invalid_argument(
              "configured_end_effector_link_invalid: link '" + end_effector_link_ +
              "' does not exist in the loaded robot model");
    }
    if (!move_group_.setEndEffectorLink(end_effector_link_)) {
      throw std::invalid_argument(
              "configured_end_effector_link_invalid: MoveIt rejected link '" +
              end_effector_link_ + "' for planning group '" + planning_group_ + "'");
    }
    move_group_.setPlanningTime(planning_time_sec_);
    move_group_.setGoalPositionTolerance(position_tolerance_);
    move_group_.setGoalOrientationTolerance(orientation_tolerance_);
    move_group_.setNumPlanningAttempts(num_planning_attempts_);
    if (!planner_id_.empty()) {
      move_group_.setPlannerId(planner_id_);
    }
  }

  void create_diagnostic_publishers()
  {
    success_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(success_topic_, 10);
    status_publisher_ = node_->create_publisher<std_msgs::msg::String>(status_topic_, 10);
    duration_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(duration_topic_, 10);
    stage_status_publisher_ =
      node_->create_publisher<std_msgs::msg::String>(stage_status_topic_, 10);
    stage_success_publisher_ =
      node_->create_publisher<std_msgs::msg::Bool>(stage_success_topic_, 10);
    stage_duration_publisher_ =
      node_->create_publisher<std_msgs::msg::Float64>(stage_duration_topic_, 10);
    trajectory_status_publisher_ =
      node_->create_publisher<std_msgs::msg::String>(trajectory_status_topic_, 10);
  }

  void resolve_stage_names()
  {
    const bool stage_names_provided = node_->has_parameter("stage_names");
    if (stage_names_provided) {
      stage_names_ = node_->get_parameter("stage_names").as_string_array();
    } else if (node_->has_parameter("stage_names_csv") &&
      !node_->get_parameter("stage_names_csv").as_string().empty())
    {
      std::istringstream stages_csv(node_->get_parameter("stage_names_csv").as_string());
      std::string stage_name;
      while (std::getline(stages_csv, stage_name, ',')) {
        stage_names_.push_back(stage_name);
      }
    } else if (require_place_sequence_) {
      stage_names_ = {"pre_grasp", "grasp", "pre_place", "place", "retreat"};
      RCLCPP_WARN(
        node_->get_logger(),
        "require_place_sequence is deprecated; use stage_names instead.");
    } else if (require_grasp_pose_) {
      stage_names_ = {"pre_grasp", "grasp", "assembly"};
      RCLCPP_WARN(
        node_->get_logger(), "require_grasp_pose is deprecated; use stage_names instead.");
    } else {
      stage_names_ = {"pre_grasp", "assembly"};
    }

    if (stage_names_.empty()) {
      RCLCPP_WARN(node_->get_logger(), "stage_names is empty; using pre_grasp,assembly.");
      stage_names_ = {"pre_grasp", "assembly"};
    }

    std::unordered_set<std::string> seen;
    std::vector<std::string> valid_names;
    for (const auto & name : stage_names_) {
      if (name.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Ignoring an empty stage name.");
      } else if (!seen.insert(name).second) {
        RCLCPP_WARN(node_->get_logger(), "Ignoring duplicate stage '%s'.", name.c_str());
      } else {
        valid_names.push_back(name);
      }
    }
    stage_names_ = std::move(valid_names);
    if (stage_names_.empty()) {
      stage_names_ = {"pre_grasp", "assembly"};
    }

    std::ostringstream sequence;
    for (std::size_t index = 0; index < stage_names_.size(); ++index) {
      if (index > 0) {
        sequence << ',';
      }
      sequence << stage_names_[index];
    }
    stage_sequence_ = sequence.str();
  }

  void create_stages()
  {
    stages_.reserve(stage_names_.size());
    for (const auto & name : stage_names_) {
      stages_.push_back(std::make_unique<StageConfig>());
      auto & stage = *stages_.back();
      stage.name = name;
      stage.pose_topic = declare_parameter<std::string>(
        name + "_topic", "/panda_" + name + "_pose");
      stage.trajectory_topic = declare_parameter<std::string>(
        name + "_trajectory_topic", "/" + name + "_trajectory");
      stage.trajectory_publisher =
        node_->create_publisher<moveit_msgs::msg::RobotTrajectory>(
        stage.trajectory_topic, 10);
      StageConfig * stage_ptr = &stage;
      stage.subscription = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        stage.pose_topic, 10,
        [this, stage_ptr](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
          stage_ptr->pose = *message;
          stage_ptr->updated = true;
          try_plan_sequence();
        });
      RCLCPP_INFO(
        node_->get_logger(), "Configured stage '%s': pose='%s', trajectory='%s'.",
        stage.name.c_str(), stage.pose_topic.c_str(), stage.trajectory_topic.c_str());
    }
  }

  void try_plan_sequence()
  {
    const bool all_ready = std::all_of(
      stages_.begin(), stages_.end(),
      [](const std::unique_ptr<StageConfig> & stage) {
        return stage->pose.has_value() && stage->updated;
      });
    if (!all_ready) {
      return;
    }
    for (auto & stage : stages_) {
      stage->updated = false;
    }
    plan_sequence();
  }

  void plan_sequence()
  {
    const std::size_t requested_stage_count = stages_.size();
    std::size_t planned_stage_count = 0;
    double total_duration_ms = 0.0;
    std::optional<moveit_msgs::msg::RobotState> next_start_state;

    set_pre_grasp_start_state();
    for (std::size_t index = 0; index < stages_.size(); ++index) {
      auto & stage = *stages_[index];
      if (index > 0) {
        if (!next_start_state.has_value()) {
          publish_skipped_trajectories(index, "invalid_previous_trajectory");
          publish_result(
            false, "failure", stage.name, planned_stage_count, requested_stage_count,
            total_duration_ms, "invalid_previous_trajectory");
          return;
        }
        move_group_.setStartState(next_start_state.value());
      }

      set_pose_target(stage.pose.value());
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto result = plan_stage(plan);
      total_duration_ms += result.duration_ms;
      move_group_.clearPoseTargets();
      publish_stage_result(
        stage.name, index, requested_stage_count, result.succeeded, result.duration_ms);

      if (!result.succeeded) {
        publish_trajectory_status(
          "skipped", stage.name, index, requested_stage_count, stage.trajectory_topic,
          nullptr, "planning_failed");
        publish_skipped_trajectories(index + 1, stage.name + "_failed");
        publish_result(
          false, "failure", stage.name, planned_stage_count, requested_stage_count,
          total_duration_ms, "planning_failed");
        return;
      }

      ++planned_stage_count;
      publish_trajectory(stage, index, requested_stage_count, plan.trajectory);
      next_start_state = final_state_from_plan(plan);
      if (!next_start_state.has_value()) {
        const bool next_stage_available = index + 1 < stages_.size();
        if (next_stage_available) {
          publish_skipped_trajectories(index + 1, "invalid_previous_trajectory");
        }
        publish_result(
          false, "failure",
          next_stage_available ? stages_[index + 1]->name : stage.name,
          planned_stage_count,
          requested_stage_count, total_duration_ms, "invalid_previous_trajectory");
        return;
      }
    }

    publish_result(
      true, "success", "none", planned_stage_count, requested_stage_count,
      total_duration_ms, "");
    RCLCPP_INFO(
      node_->get_logger(),
      "Sequence planning succeeded for %zu stages in %.3f ms; execution remains disabled.",
      planned_stage_count, total_duration_ms);
  }

  void publish_skipped_trajectories(
    const std::size_t first_index, const std::string & reason)
  {
    for (std::size_t index = first_index; index < stages_.size(); ++index) {
      const auto & stage = *stages_[index];
      publish_trajectory_status(
        "skipped", stage.name, index, stages_.size(), stage.trajectory_topic, nullptr, reason);
    }
  }

  void set_pose_target(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!pose.header.frame_id.empty()) {
      move_group_.setPoseReferenceFrame(pose.header.frame_id);
    }
    move_group_.setPoseTarget(pose, end_effector_link_);
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
    fixed_state.joint_state.position.assign(joint_positions.begin(), joint_positions.end());
    fixed_state.is_diff = false;
    move_group_.setStartState(fixed_state);
  }

  StagePlanResult plan_stage(
    moveit::planning_interface::MoveGroupInterface::Plan & plan)
  {
    const auto start = std::chrono::steady_clock::now();
    const bool succeeded = static_cast<bool>(move_group_.plan(plan));
    const auto end = std::chrono::steady_clock::now();
    return {
      succeeded,
      std::chrono::duration<double, std::milli>(end - start).count()
    };
  }

  static std::optional<moveit_msgs::msg::RobotState> final_state_from_plan(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan)
  {
    const auto & trajectory = plan.trajectory.joint_trajectory;
    if (trajectory.points.empty()) {
      return std::nullopt;
    }
    moveit_msgs::msg::RobotState state = plan.start_state;
    const auto & positions = trajectory.points.back().positions;
    const std::size_t count = std::min(trajectory.joint_names.size(), positions.size());
    for (std::size_t index = 0; index < count; ++index) {
      const auto existing = std::find(
        state.joint_state.name.begin(), state.joint_state.name.end(),
        trajectory.joint_names[index]);
      if (existing == state.joint_state.name.end()) {
        state.joint_state.name.push_back(trajectory.joint_names[index]);
        state.joint_state.position.push_back(positions[index]);
      } else {
        const auto state_index = static_cast<std::size_t>(
          std::distance(state.joint_state.name.begin(), existing));
        if (state.joint_state.position.size() <= state_index) {
          state.joint_state.position.resize(state_index + 1, 0.0);
        }
        state.joint_state.position[state_index] = positions[index];
      }
    }
    return state;
  }

  void publish_trajectory(
    const StageConfig & stage, const std::size_t stage_index,
    const std::size_t requested_stage_count,
    const moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    if (!publish_trajectories_) {
      publish_trajectory_status(
        "skipped", stage.name, stage_index, requested_stage_count,
        stage.trajectory_topic, &trajectory, "publishing_disabled");
      return;
    }
    stage.trajectory_publisher->publish(trajectory);
    publish_trajectory_status(
      "published", stage.name, stage_index, requested_stage_count,
      stage.trajectory_topic, &trajectory, "");
  }

  void publish_trajectory_status(
    const std::string & event, const std::string & stage,
    const std::size_t stage_index, const std::size_t requested_stage_count,
    const std::string & topic, const moveit_msgs::msg::RobotTrajectory * trajectory,
    const std::string & reason)
  {
    std_msgs::msg::String message;
    std::ostringstream status;
    status << "event=" << event << ";stage=" << stage
           << ";stage_index=" << stage_index
           << ";requested_stage_count=" << requested_stage_count
           << ";topic=" << topic;
    if (!reason.empty()) {
      status << ";reason=" << reason;
    }
    if (trajectory != nullptr) {
      status << ";point_count=" << trajectory->joint_trajectory.points.size()
             << ";joint_count=" << trajectory->joint_trajectory.joint_names.size();
    }
    status << ";execution=false";
    message.data = status.str();
    trajectory_status_publisher_->publish(message);
  }

  void publish_result(
    const bool success, const std::string & event, const std::string & failed_stage,
    const std::size_t planned_stage_count, const std::size_t requested_stage_count,
    const double total_duration_ms, const std::string & reason)
  {
    std_msgs::msg::Bool success_message;
    success_message.data = success;
    success_publisher_->publish(success_message);
    if (!publish_diagnostics_) {
      return;
    }
    std_msgs::msg::String message;
    std::ostringstream status;
    status << std::fixed << std::setprecision(6)
           << "event=" << event << ";failed_stage=" << failed_stage
           << ";planned_stage_count=" << planned_stage_count
           << ";requested_stage_count=" << requested_stage_count
           << ";stage_sequence=" << stage_sequence_
           << ";total_duration_ms=" << total_duration_ms
           << ";start_state_mode=" << start_state_mode_
           << ";end_effector_link=" << end_effector_link_;
    if (!reason.empty()) {
      status << ";reason=" << reason;
    }
    status << ";execution=false";
    message.data = status.str();
    status_publisher_->publish(message);
    std_msgs::msg::Float64 duration_message;
    duration_message.data = total_duration_ms;
    duration_publisher_->publish(duration_message);
  }

  void publish_stage_result(
    const std::string & stage, const std::size_t stage_index,
    const std::size_t requested_stage_count, const bool success,
    const double duration_ms)
  {
    std_msgs::msg::Bool success_message;
    success_message.data = success;
    stage_success_publisher_->publish(success_message);
    if (!publish_diagnostics_) {
      return;
    }
    std_msgs::msg::String message;
    std::ostringstream status;
    status << std::fixed << std::setprecision(6)
           << "event=" << (success ? "success" : "failure")
           << ";stage=" << stage << ";stage_index=" << stage_index
           << ";requested_stage_count=" << requested_stage_count
           << ";duration_ms=" << duration_ms
           << ";planning_group=" << planning_group_
           << ";end_effector_link=" << end_effector_link_
           << ";planner_id=" << (planner_id_.empty() ? "<default>" : planner_id_)
           << ";num_planning_attempts=" << num_planning_attempts_
           << ";planning_time_sec=" << planning_time_sec_
           << ";position_tolerance=" << position_tolerance_
           << ";orientation_tolerance=" << orientation_tolerance_
           << ";start_state_mode=" << start_state_mode_
           << ";execution=false";
    message.data = status.str();
    stage_status_publisher_->publish(message);
    std_msgs::msg::Float64 duration_message;
    duration_message.data = duration_ms;
    stage_duration_publisher_->publish(duration_message);
  }

  rclcpp::Node::SharedPtr node_;
  std::string success_topic_;
  std::string status_topic_;
  std::string duration_topic_;
  std::string stage_status_topic_;
  std::string stage_success_topic_;
  std::string stage_duration_topic_;
  std::string trajectory_status_topic_;
  bool publish_diagnostics_;
  bool publish_trajectories_;
  bool require_grasp_pose_;
  bool require_place_sequence_;
  std::string planning_group_;
  std::string end_effector_link_;
  std::string planner_id_;
  int num_planning_attempts_;
  double planning_time_sec_;
  double position_tolerance_;
  double orientation_tolerance_;
  std::string start_state_mode_;
  std::vector<std::string> stage_names_;
  std::string stage_sequence_;
  std::vector<std::unique_ptr<StageConfig>> stages_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr success_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr duration_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stage_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stage_success_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stage_duration_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trajectory_status_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "assembly_sequence_planning_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  std::shared_ptr<AssemblySequencePlanningNode> planner;
  try {
    planner = std::make_shared<AssemblySequencePlanningNode>(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(node->get_logger(), "%s", exception.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::spin(node);
  planner.reset();
  rclcpp::shutdown();
  return 0;
}
