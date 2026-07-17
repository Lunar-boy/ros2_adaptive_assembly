#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include "adaptive_assembly_planning/linear_path_validation.hpp"
#include "adaptive_assembly_planning/grasp_clearance_validation.hpp"
#include "adaptive_assembly_planning/planning_contract.hpp"

namespace aap = adaptive_assembly_planning;

class AssemblySequencePlanningNode
{
public:
  explicit AssemblySequencePlanningNode(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    success_topic_(param<std::string>("success_topic", "/assembly_sequence_plan_success")),
    status_topic_(param<std::string>("status_topic", "/assembly_sequence_planning_status")),
    duration_topic_(param<std::string>("duration_topic",
    "/assembly_sequence_planning_duration_ms")),
    stage_status_topic_(param<std::string>("stage_status_topic",
    "/assembly_sequence_stage_status")),
    stage_success_topic_(param<std::string>("stage_success_topic",
    "/assembly_sequence_stage_success")),
    stage_duration_topic_(param<std::string>("stage_duration_topic",
    "/assembly_sequence_stage_duration_ms")),
    trajectory_status_topic_(param<std::string>("trajectory_status_topic",
    "/assembly_sequence_trajectory_status")),
    plan_lock_status_topic_(param<std::string>("plan_lock_status_topic",
    "/assembly_sequence_plan_lock_status")),
    plan_phase_(param<std::string>("plan_phase", "sequence")),
    publish_diagnostics_(param<bool>("publish_diagnostics", true)),
    publish_trajectories_(param<bool>("publish_trajectories", true)),
    lock_after_successful_sequence_(param<bool>("lock_after_successful_sequence", false)),
    require_dynamic_target_scene_ready_(param<bool>("require_dynamic_target_scene_ready", false)),
    grasp_clearance_config_{
      param<bool>("require_grasp_clearance_validation", false),
      param<double>("grasp_min_disallowed_clearance", 0.005),
      param<std::string>("grasp_clearance_target_object_id", "target_object"),
      split_csv_vector(param<std::string>("grasp_allowed_contact_links_csv",
        "panda_leftfinger,panda_rightfinger")),
      param<double>("grasp_synthetic_finger_open_position", 0.040),
      param<double>("grasp_synthetic_finger_closed_position", 0.000),
      param<double>("grasp_synthetic_finger_step", 0.001)},
    planning_group_(param<std::string>("planning_group", "panda_arm")),
    end_effector_link_(param<std::string>("end_effector_link", "panda_link8")),
    num_planning_attempts_(param<int>("num_planning_attempts", 1)),
    planning_time_sec_(param<double>("planning_time_sec", 5.0)),
    start_state_mode_(param<std::string>("start_state_mode", "current")),
    require_fresh_current_start_state_(param<bool>("require_fresh_current_start_state", false)),
    current_start_state_freshness_sec_(param<double>("current_start_state_freshness_sec", 0.5)),
    default_profile_{
      param<std::string>("default_planning_pipeline_id", "ompl"),
      param<std::string>("default_planner_id", param<std::string>("planner_id", "")),
      param<double>("position_tolerance", 0.01),
      param<double>("orientation_tolerance", 0.10),
      param<double>("max_velocity_scaling_factor", 1.0),
      param<double>("max_acceleration_scaling_factor", 1.0), false},
    linear_profile_{
      param<std::string>("linear_planning_pipeline_id", "pilz_industrial_motion_planner"),
      param<std::string>("linear_planner_id", "LIN"),
      param<double>("linear_position_tolerance", 0.002),
      param<double>("linear_orientation_tolerance", 0.01),
      param<double>("linear_max_velocity_scaling_factor", 0.05),
      param<double>("linear_max_acceleration_scaling_factor", 0.05), true},
    snapshot_limits_{
      param<double>("pose_snapshot_max_stamp_skew_sec", 0.20),
      param<double>("grasp_approach_min_distance", 0.05),
      param<double>("grasp_approach_max_distance", 0.30),
      param<double>("grasp_approach_max_lateral_offset", 0.002),
      param<double>("grasp_approach_max_orientation_difference", 0.01),
      param<double>("pose_quaternion_norm_tolerance", 1.0e-3)},
    linear_limits_{
      param<double>("linear_max_lateral_deviation", 0.002),
      param<double>("linear_max_orientation_deviation", 0.01),
      param<double>("linear_max_endpoint_position_error", 0.002),
      param<double>("linear_max_endpoint_orientation_error", 0.01),
      param<double>("linear_max_path_length_ratio", 1.02),
      param<double>("linear_progress_tolerance", 1.0e-4),
      param<double>("linear_overshoot_tolerance", 1.0e-3)},
    move_group_(node_, planning_group_)
  {
    next_plan_id_ = static_cast<std::uint64_t>(param<int>("initial_plan_id", 0));
    validate_parameters();
    resolve_stage_names();
    linear_stage_names_ = split_csv(param<std::string>("linear_stage_names_csv", ""));
    configure_move_group_once();
    configure_planning_scene_monitor();
    create_publishers();
    joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      param<std::string>("joint_state_topic", "/joint_states"), 20,
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        if (message->name.size() == message->position.size() &&
        std::all_of(message->position.begin(), message->position.end(), [](double value) {
          return std::isfinite(value);
          }))
        {
          latest_joint_state_ = *message;
          latest_joint_state_time_ = node_->now();
          try_plan_sequence();
        }
      });
    create_stages();
    if (grasp_clearance_config_.required) {
      scene_ready_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(50), [this]() {try_plan_sequence();});
    }
    if (require_dynamic_target_scene_ready_) {
      dynamic_scene_ready_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
        param<std::string>("dynamic_target_scene_ready_topic",
        "/physical_target_planning_scene_ready"),
        rclcpp::QoS(1).transient_local().reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr message) {
          dynamic_scene_ready_ = message->data;
          try_plan_sequence();
        });
    } else {
      dynamic_scene_ready_ = true;
    }
    publish_lock_status("waiting", 0, false, 0, "waiting_for_inputs");
    RCLCPP_INFO(
      node_->get_logger(),
      "Sequence planner ready: stages='%s', linear_stages='%s', default_pipeline='%s', "
      "linear_pipeline='%s', linear_planner='%s', lock_after_success=%s.",
      stage_sequence_.c_str(), join(linear_stage_names_).c_str(),
      default_profile_.planning_pipeline_id.c_str(), linear_profile_.planning_pipeline_id.c_str(),
      linear_profile_.planner_id.c_str(), lock_after_successful_sequence_ ? "true" : "false");
  }

private:
  enum class State {WAITING_FOR_INPUTS, PLANNING, LOCKED};

  struct Stage
  {
    std::string name;
    std::string pose_topic;
    std::string trajectory_topic;
    std::optional<geometry_msgs::msg::PoseStamped> latest_pose;
    std::uint64_t latest_pose_generation{0};
    std::uint64_t consumed_generation{0};
    rclcpp::Time latest_receive_time;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription;
    rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr trajectory_publisher;
  };

  struct Candidate
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    aap::StagePlanningProfile profile;
    aap::LinearPathMetrics linear_metrics;
    aap::GraspClearanceMetrics clearance_metrics;
    double duration_ms{0.0};
  };

  template<typename T>
  T param(const std::string & name, const T & default_value)
  {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<T>(name, default_value);
    }
    return node_->get_parameter(name).get_value<T>();
  }

  static std::unordered_set<std::string> split_csv(const std::string & text)
  {
    std::unordered_set<std::string> values;
    std::istringstream stream(text);
    std::string value;
    while (std::getline(stream, value, ',')) {
      value.erase(0, value.find_first_not_of(" \t"));
      if (!value.empty()) {
        value.erase(value.find_last_not_of(" \t") + 1);
        values.insert(value);
      }
    }
    return values;
  }

  static std::vector<std::string> split_csv_vector(const std::string & text)
  {
    const auto values = split_csv(text);
    std::vector<std::string> result(values.begin(), values.end());
    std::sort(result.begin(), result.end());
    return result;
  }

  static std::string join(const std::unordered_set<std::string> & values)
  {
    std::vector<std::string> sorted(values.begin(), values.end());
    std::sort(sorted.begin(), sorted.end());
    std::ostringstream out;
    for (std::size_t i = 0; i < sorted.size(); ++i) {
      if (i) {out << ',';}
      out << sorted[i];
    }
    return out.str();
  }

  void validate_parameters()
  {
    if (num_planning_attempts_ < 1 || planning_time_sec_ <= 0.0) {
      throw std::invalid_argument("invalid_planning_configuration");
    }
    const auto validate_profile = [](const aap::StagePlanningProfile & p) {
        return !p.planning_pipeline_id.empty() && p.position_tolerance > 0.0 &&
               p.orientation_tolerance > 0.0 && p.max_velocity_scaling_factor > 0.0 &&
               p.max_velocity_scaling_factor <= 1.0 && p.max_acceleration_scaling_factor > 0.0 &&
               p.max_acceleration_scaling_factor <= 1.0;
      };
    if (!validate_profile(default_profile_) || !validate_profile(linear_profile_) ||
      linear_profile_.planner_id.empty())
    {
      throw std::invalid_argument("invalid_stage_planning_profile");
    }
    if (start_state_mode_ != "current" && start_state_mode_ != "fixed") {
      throw std::invalid_argument("invalid_start_state_mode");
    }
    if (current_start_state_freshness_sec_ <= 0.0) {
      throw std::invalid_argument("invalid_current_start_state_freshness");
    }
    if (grasp_clearance_config_.required &&
      (!(grasp_clearance_config_.minimum_disallowed_clearance >= 0.005) ||
      grasp_clearance_config_.target_object_id.empty() ||
      grasp_clearance_config_.allowed_contact_links !=
      std::vector<std::string>({"panda_leftfinger", "panda_rightfinger"})))
    {
      throw std::invalid_argument("invalid_grasp_clearance_configuration");
    }
  }

  void resolve_stage_names()
  {
    if (node_->has_parameter("stage_names")) {
      stage_names_ = node_->get_parameter("stage_names").as_string_array();
    } else {
      const auto names = split_csv(param<std::string>(
        "stage_names_csv", "pre_grasp,grasp,lift,pre_place,place,retreat"));
      const std::vector<std::string> canonical = {
        "pre_grasp", "grasp", "lift", "pre_place", "place", "retreat", "assembly"};
      for (const auto & name : canonical) {
        if (names.count(name)) {stage_names_.push_back(name);}
      }
    }
    if (stage_names_.empty()) {throw std::invalid_argument("empty_stage_sequence");}
    std::unordered_set<std::string> unique;
    std::ostringstream sequence;
    for (std::size_t i = 0; i < stage_names_.size(); ++i) {
      if (stage_names_[i].empty() || !unique.insert(stage_names_[i]).second) {
        throw std::invalid_argument("invalid_stage_sequence");
      }
      if (i) {sequence << ',';}
      sequence << stage_names_[i];
    }
    stage_sequence_ = sequence.str();
  }

  void configure_move_group_once()
  {
    robot_model_ = move_group_.getRobotModel();
    if (!robot_model_ || !robot_model_->hasLinkModel(end_effector_link_)) {
      throw std::invalid_argument("configured_end_effector_link_invalid");
    }
    if (!move_group_.setEndEffectorLink(end_effector_link_)) {
      throw std::invalid_argument("configured_end_effector_link_invalid");
    }
    move_group_.setPlanningTime(planning_time_sec_);
    move_group_.setNumPlanningAttempts(num_planning_attempts_);
  }

  void configure_planning_scene_monitor()
  {
    if (!grasp_clearance_config_.required) {return;}
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, "robot_description", "assembly_sequence_clearance_monitor");
    if (!planning_scene_monitor_->getPlanningScene()) {
      throw std::invalid_argument("grasp_clearance_scene_unavailable");
    }
    planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
  }

  planning_scene::PlanningSceneConstPtr frozen_scene_snapshot() const
  {
    if (!grasp_clearance_config_.required) {return nullptr;}
    if (!planning_scene_monitor_) {return nullptr;}
    planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);
    if (!locked_scene) {return nullptr;}
    auto snapshot = locked_scene->diff();
    snapshot->decoupleParent();
    return snapshot;
  }

  void create_publishers()
  {
    success_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(success_topic_, 10);
    status_publisher_ = node_->create_publisher<std_msgs::msg::String>(status_topic_, 10);
    duration_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(duration_topic_, 10);
    stage_status_publisher_ = node_->create_publisher<std_msgs::msg::String>(stage_status_topic_,
      10);
    stage_success_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(stage_success_topic_,
      10);
    stage_duration_publisher_ =
      node_->create_publisher<std_msgs::msg::Float64>(stage_duration_topic_, 10);
    trajectory_status_publisher_ =
      node_->create_publisher<std_msgs::msg::String>(trajectory_status_topic_, 10);
    lock_status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
      plan_lock_status_topic_, rclcpp::QoS(10).reliable().durability_volatile());
  }

  void create_stages()
  {
    for (const auto & name : stage_names_) {
      auto stage = std::make_unique<Stage>();
      stage->name = name;
      stage->pose_topic = param<std::string>(name + "_topic", "/panda_" + name + "_pose");
      stage->trajectory_topic = param<std::string>(name + "_trajectory_topic",
        "/" + name + "_trajectory");
      stage->trajectory_publisher = node_->create_publisher<moveit_msgs::msg::RobotTrajectory>(
        stage->trajectory_topic, 10);
      Stage * raw = stage.get();
      stage->subscription = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        stage->pose_topic, 10,
        [this, raw](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
          raw->latest_pose = *message;
          ++raw->latest_pose_generation;
          raw->latest_receive_time = node_->now();
          if (state_ == State::LOCKED) {
            publish_update_ignored(raw->name);
            return;
          }
          try_plan_sequence();
        });
      stages_.push_back(std::move(stage));
    }
  }

  void publish_update_ignored(const std::string & stage)
  {
    const auto now = std::chrono::steady_clock::now();
    if (last_ignored_status_.time_since_epoch().count() != 0 &&
      now - last_ignored_status_ < std::chrono::seconds(2)) {return;}
    last_ignored_status_ = now;
    publish_lock_status("update_ignored", locked_plan_id_, true, stages_.size(),
      "plan_already_locked", stage);
  }

  bool inputs_ready() const
  {
    return dynamic_scene_ready_ && clearance_scene_ready() && current_start_state_ready() &&
           std::all_of(stages_.begin(), stages_.end(), [](const auto & s) {
               return s->latest_pose.has_value() &&
                      s->latest_pose_generation > s->consumed_generation;
      });
  }

  bool current_start_state_ready() const
  {
    if (start_state_mode_ != "current" || !require_fresh_current_start_state_) {return true;}
    return latest_joint_state_.has_value() &&
           (node_->now() - latest_joint_state_time_).seconds() >= 0.0 &&
           (node_->now() - latest_joint_state_time_).seconds() <=
           current_start_state_freshness_sec_;
  }

  bool clearance_scene_ready() const
  {
    if (!grasp_clearance_config_.required) {return true;}
    if (!planning_scene_monitor_) {return false;}
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    return scene && scene->getWorld() &&
           scene->getWorld()->hasObject(grasp_clearance_config_.target_object_id);
  }

  void try_plan_sequence()
  {
    if (state_ != State::WAITING_FOR_INPUTS || !inputs_ready()) {return;}
    std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> snapshot;
    snapshot.reserve(stages_.size());
    for (const auto & stage : stages_) {snapshot.emplace_back(stage->name, *stage->latest_pose);}
    const std::string snapshot_error = aap::validate_snapshot(snapshot, snapshot_limits_);
    for (auto & stage : stages_) {stage->consumed_generation = stage->latest_pose_generation;}
    const std::uint64_t plan_id = ++next_plan_id_;
    snapshot_stamp_ns_ = node_->now().nanoseconds();
    if (!snapshot_error.empty()) {
      publish_failure(plan_id, "none", 0, 0.0, snapshot_error);
      return;
    }
    const auto scene_snapshot = frozen_scene_snapshot();
    if (grasp_clearance_config_.required && !scene_snapshot) {
      publish_failure(plan_id, "grasp", 0, 0.0, "grasp_clearance_scene_unavailable");
      return;
    }
    plan_sequence(plan_id, snapshot, scene_snapshot);
  }

  void apply_profile(const aap::StagePlanningProfile & profile)
  {
    move_group_.setPlanningPipelineId(profile.planning_pipeline_id);
    move_group_.setPlannerId(profile.planner_id);
    move_group_.setGoalPositionTolerance(profile.position_tolerance);
    move_group_.setGoalOrientationTolerance(profile.orientation_tolerance);
    move_group_.setMaxVelocityScalingFactor(profile.max_velocity_scaling_factor);
    move_group_.setMaxAccelerationScalingFactor(profile.max_acceleration_scaling_factor);
  }

  bool set_initial_start_state()
  {
    if (start_state_mode_ == "current") {
      if (latest_joint_state_) {
        moveit_msgs::msg::RobotState state;
        state.joint_state = *latest_joint_state_;
        state.is_diff = true;
        move_group_.setStartState(state);
        return true;
      }
      if (require_fresh_current_start_state_) {return false;}
      move_group_.setStartStateToCurrentState();
      return true;
    }
    constexpr std::array<const char *, 7> names = {
      "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
      "panda_joint5", "panda_joint6", "panda_joint7"};
    constexpr std::array<double, 7> positions = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
    moveit_msgs::msg::RobotState state;
    state.joint_state.name.assign(names.begin(), names.end());
    state.joint_state.position.assign(positions.begin(), positions.end());
    move_group_.setStartState(state);
    return true;
  }

  std::optional<moveit_msgs::msg::RobotState> final_state(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const std::optional<moveit_msgs::msg::RobotState> & requested_start_state) const
  {
    const auto & jt = plan.trajectory.joint_trajectory;
    if (jt.points.empty() || jt.joint_names.size() != jt.points.back().positions.size()) {
      return std::nullopt;
    }
    try {
      moveit::core::RobotState complete_state(robot_model_);
      const auto & base = requested_start_state.value_or(plan.start_state);
      moveit::core::robotStateMsgToRobotState(base, complete_state, true);
      complete_state.setVariablePositions(jt.joint_names, jt.points.back().positions);
      const std::vector<double> zero(robot_model_->getVariableCount(), 0.0);
      complete_state.setVariableVelocities(zero);
      complete_state.setVariableEffort(zero);
      complete_state.update();
      moveit_msgs::msg::RobotState state;
      moveit::core::robotStateToRobotStateMsg(complete_state, state, true);
      state.is_diff = false;
      return state;
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }

  std::optional<std::vector<geometry_msgs::msg::Pose>> fk_samples(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan) const
  {
    moveit::core::RobotState state(robot_model_);
    try {
      moveit::core::robotStateMsgToRobotState(plan.start_state, state, true);
      std::vector<geometry_msgs::msg::Pose> samples;
      const auto & jt = plan.trajectory.joint_trajectory;
      for (const auto & point : jt.points) {
        if (point.positions.size() != jt.joint_names.size() ||
          !std::all_of(point.positions.begin(), point.positions.end(), [](double v) {
            return std::isfinite(v);
                                                                                                             }))
        {return std::nullopt;}
        state.setVariablePositions(jt.joint_names, point.positions);
        state.updateLinkTransforms();
        const Eigen::Isometry3d & transform = state.getGlobalLinkTransform(end_effector_link_);
        const Eigen::Quaterniond q(transform.rotation());
        geometry_msgs::msg::Pose pose;
        pose.position.x = transform.translation().x();
        pose.position.y = transform.translation().y();
        pose.position.z = transform.translation().z();
        pose.orientation.x = q.x(); pose.orientation.y = q.y();
        pose.orientation.z = q.z(); pose.orientation.w = q.w();
        samples.push_back(pose);
      }
      return samples;
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }

  void plan_sequence(
    const std::uint64_t plan_id,
    const std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> & snapshot,
    const planning_scene::PlanningSceneConstPtr & scene_snapshot)
  {
    last_clearance_metrics_ = {};
    state_ = State::PLANNING;
    publish_lock_status("planning_started", plan_id, false, 0, "");
    std::vector<Candidate> candidates;
    std::optional<moveit_msgs::msg::RobotState> next_start;
    geometry_msgs::msg::Pose pre_grasp_fk;
    bool have_pre_grasp_fk = false;
    double total_ms = 0.0;
    if (!set_initial_start_state()) {
      publish_failure(plan_id, stages_.front()->name, 0, 0.0,
        "current_start_state_unavailable");
      return;
    }

    for (std::size_t i = 0; i < stages_.size(); ++i) {
      const auto & stage = *stages_[i];
      const auto profile = aap::resolve_stage_profile(
        stage.name, linear_stage_names_, default_profile_, linear_profile_);
      apply_profile(profile);
      if (i > 0) {
        if (!next_start) {
          publish_failure(plan_id, stage.name, candidates.size(), total_ms,
            "invalid_previous_trajectory"); return;
        }
        move_group_.setStartState(*next_start);
      }
      move_group_.setPoseReferenceFrame(snapshot[i].second.header.frame_id);
      move_group_.setPoseTarget(snapshot[i].second, end_effector_link_);
      Candidate candidate;
      candidate.profile = profile;
      const auto begin = std::chrono::steady_clock::now();
      const bool success = static_cast<bool>(move_group_.plan(candidate.plan));
      candidate.duration_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count();
      total_ms += candidate.duration_ms;
      move_group_.clearPoseTargets();
      if (!success) {
        const std::string reason =
          profile.require_linear_validation ? "linear_planning_failed" : "planning_failed";
        publish_stage(candidate, stage.name, i, plan_id, false, reason);
        publish_failure(plan_id, stage.name, candidates.size(), total_ms, reason);
        return;
      }
      next_start = final_state(candidate.plan, next_start);
      if (!next_start) {
        publish_stage(candidate, stage.name, i, plan_id, false, "invalid_trajectory_final_state");
        publish_failure(plan_id, stage.name, candidates.size(), total_ms,
          "invalid_trajectory_final_state");
        return;
      }
      const auto samples = fk_samples(candidate.plan);
      if (!samples || samples->empty()) {
        publish_stage(candidate, stage.name, i, plan_id, false, "linear_path_invalid_fk");
        publish_failure(plan_id, stage.name, candidates.size(), total_ms, "linear_path_invalid_fk");
        return;
      }
      if (stage.name == "pre_grasp") {
        pre_grasp_fk = samples->back();
        have_pre_grasp_fk = true;
      }
      if (profile.require_linear_validation) {
        if (!have_pre_grasp_fk) {
          publish_failure(plan_id, stage.name, candidates.size(), total_ms,
            "linear_path_missing_pre_grasp"); return;
        }
        candidate.linear_metrics = aap::validate_linear_path(
          pre_grasp_fk, snapshot[i].second.pose, *samples, linear_limits_);
        if (!candidate.linear_metrics.valid) {
          publish_stage(candidate, stage.name, i, plan_id, false, candidate.linear_metrics.reason);
          publish_failure(plan_id, stage.name, candidates.size(), total_ms,
            candidate.linear_metrics.reason);
          return;
        }
        if (grasp_clearance_config_.required) {
          candidate.clearance_metrics = aap::evaluate_grasp_clearance(
            scene_snapshot, robot_model_, candidate.plan.start_state,
            candidate.plan.trajectory, grasp_clearance_config_, end_effector_link_);
          last_clearance_metrics_ = candidate.clearance_metrics;
          if (!candidate.clearance_metrics.valid) {
            publish_stage(
              candidate, stage.name, i, plan_id, false,
              candidate.clearance_metrics.reason);
            publish_failure(
              plan_id, stage.name, candidates.size(), total_ms,
              candidate.clearance_metrics.reason);
            return;
          }
        }
      }
      publish_stage(candidate, stage.name, i, plan_id, true, "");
      candidates.push_back(std::move(candidate));
    }

    if (publish_trajectories_) {
      for (std::size_t i = 0; i < stages_.size(); ++i) {
        stages_[i]->trajectory_publisher->publish(candidates[i].plan.trajectory);
        publish_trajectory_status("published", *stages_[i], i, plan_id, candidates[i], "");
      }
    }
    publish_result(true, plan_id, "none", candidates.size(), total_ms, "");
    if (lock_after_successful_sequence_) {
      state_ = State::LOCKED;
      locked_plan_id_ = plan_id;
      publish_lock_status("locked", plan_id, true, candidates.size(), "");
    } else {
      state_ = State::WAITING_FOR_INPUTS;
    }
  }

  std::string profile_fields(const aap::StagePlanningProfile & p) const
  {
    std::ostringstream out;
    out << ";planning_pipeline_id=" << p.planning_pipeline_id
        << ";planner_id=" << (p.planner_id.empty() ? "<default>" : p.planner_id)
        << ";position_tolerance=" << p.position_tolerance
        << ";orientation_tolerance=" << p.orientation_tolerance
        << ";max_velocity_scaling_factor=" << p.max_velocity_scaling_factor
        << ";max_acceleration_scaling_factor=" << p.max_acceleration_scaling_factor
        << ";require_linear_validation=" << (p.require_linear_validation ? "true" : "false");
    return out.str();
  }

  std::string metric_fields(const aap::LinearPathMetrics & m) const
  {
    if (m.direct_distance == 0.0 && m.reason.empty()) {return "";}
    std::ostringstream out;
    out << ";direct_distance=" << m.direct_distance
        << ";sampled_cartesian_path_length=" << m.sampled_cartesian_path_length
        << ";path_length_ratio=" << m.path_length_ratio
        << ";max_lateral_deviation=" << m.max_lateral_deviation
        << ";max_orientation_deviation=" << m.max_orientation_deviation
        << ";endpoint_position_error=" << m.endpoint_position_error
        << ";endpoint_orientation_error=" << m.endpoint_orientation_error
        << ";minimum_progress=" << m.minimum_progress
        << ";maximum_progress=" << m.maximum_progress
        << ";monotonic_progress=" << (m.monotonic_progress ? "true" : "false");
    return out.str();
  }

  std::string clearance_fields(const aap::GraspClearanceMetrics & metrics) const
  {
    if (!std::isfinite(metrics.grasp_height_offset) && metrics.reason.empty()) {return "";}
    std::ostringstream out;
    out << ";grasp_height_offset=" << metrics.grasp_height_offset
        << ";minimum_disallowed_clearance=" << metrics.minimum_disallowed_clearance
        << ";nearest_disallowed_link=" <<
      (metrics.nearest_disallowed_link.empty() ? "unavailable" :
    metrics.nearest_disallowed_link)
        << ";disallowed_collision_count=" << metrics.disallowed_collision_count
        << ";left_finger_target_geometry_valid=" <<
      (metrics.left_finger_target_geometry_valid ? "true" : "false")
        << ";right_finger_target_geometry_valid=" <<
      (metrics.right_finger_target_geometry_valid ? "true" : "false")
        << ";grasp_clearance_valid=" << (metrics.grasp_clearance_valid ? "true" : "false");
    return out.str();
  }

  void publish_stage(
    const Candidate & candidate, const std::string & stage, std::size_t index,
    std::uint64_t plan_id, bool success, const std::string & reason)
  {
    std_msgs::msg::Bool success_msg; success_msg.data = success;
    stage_success_publisher_->publish(success_msg);
    if (!publish_diagnostics_) {return;}
    std_msgs::msg::String message;
    std::ostringstream out;
    out << std::fixed << std::setprecision(6)
        << "event=" << (success ? "success" : "failure") << ";stage=" << stage
        << ";stage_index=" << index << ";requested_stage_count=" << stages_.size()
        << ";plan_id=" << plan_id << ";duration_ms=" << candidate.duration_ms
        << ";planning_group=" << planning_group_ << ";end_effector_link=" << end_effector_link_
        << profile_fields(candidate.profile) << metric_fields(candidate.linear_metrics)
        << clearance_fields(candidate.clearance_metrics);
    if (!reason.empty()) {out << ";reason=" << reason;}
    out << ";execution=false";
    message.data = out.str(); stage_status_publisher_->publish(message);
    RCLCPP_INFO(node_->get_logger(), "Stage planning status: %s", message.data.c_str());
    std_msgs::msg::Float64 duration; duration.data = candidate.duration_ms;
    stage_duration_publisher_->publish(duration);
  }

  void publish_trajectory_status(
    const std::string & event, const Stage & stage, std::size_t index,
    std::uint64_t plan_id, const Candidate & candidate, const std::string & reason)
  {
    std_msgs::msg::String message;
    std::ostringstream out;
    out << std::fixed << std::setprecision(6) << "event=" << event << ";stage=" << stage.name
        << ";stage_index=" << index << ";requested_stage_count=" << stages_.size()
        << ";topic=" << stage.trajectory_topic << ";plan_id=" << plan_id
        << ";point_count=" << candidate.plan.trajectory.joint_trajectory.points.size()
        << ";joint_count=" << candidate.plan.trajectory.joint_trajectory.joint_names.size()
        << profile_fields(candidate.profile) << metric_fields(candidate.linear_metrics)
        << clearance_fields(candidate.clearance_metrics);
    if (!reason.empty()) {out << ";reason=" << reason;}
    out << ";execution=false"; message.data = out.str();
    trajectory_status_publisher_->publish(message);
  }

  void publish_failure(
    std::uint64_t plan_id, const std::string & stage, std::size_t planned,
    double total_ms, const std::string & reason)
  {
    publish_result(false, plan_id, stage, planned, total_ms, reason);
    publish_lock_status("failure", plan_id, false, planned, reason);
    RCLCPP_WARN(
      node_->get_logger(), "Sequence planning failed: plan_id=%lu stage='%s' reason='%s'.",
      static_cast<unsigned long>(plan_id), stage.c_str(), reason.c_str());
    state_ = State::WAITING_FOR_INPUTS;
  }

  void publish_result(
    bool success, std::uint64_t plan_id, const std::string & failed_stage,
    std::size_t planned, double total_ms, const std::string & reason)
  {
    std_msgs::msg::Bool success_msg; success_msg.data = success;
    success_publisher_->publish(success_msg);
    std_msgs::msg::String message;
    std::ostringstream out;
    out << std::fixed << std::setprecision(6) << "event=" << (success ? "success" : "failure")
        << ";plan_id=" << plan_id << ";failed_stage=" << failed_stage
        << ";planned_stage_count=" << planned << ";requested_stage_count=" << stages_.size()
        << ";stage_sequence=" << stage_sequence_ << ";total_duration_ms=" << total_ms
        << ";start_state_mode=" << start_state_mode_ << ";end_effector_link=" << end_effector_link_
        << clearance_fields(last_clearance_metrics_);
    if (!reason.empty()) {out << ";reason=" << reason;}
    out << ";execution=false"; message.data = out.str(); status_publisher_->publish(message);
    std_msgs::msg::Float64 duration; duration.data = total_ms;
    duration_publisher_->publish(duration);
  }

  void publish_lock_status(
    const std::string & event, std::uint64_t plan_id, bool locked,
    std::size_t planned_count, const std::string & reason,
    const std::string & updated_stage = "")
  {
    std_msgs::msg::String message;
    std::ostringstream out;
    out << "event=" << event << ";mode=sequence_plan_lock;phase=" << plan_phase_
        << ";plan_id=" << plan_id
        << ";stage_sequence=" << stage_sequence_ << ";snapshot_stamp_ns=" << snapshot_stamp_ns_
        << ";locked=" << (locked ? "true" : "false")
        << ";planned_stage_count=" << planned_count;
    if (!updated_stage.empty()) {out << ";stage=" << updated_stage;}
    if (!reason.empty()) {out << ";reason=" << reason;}
    out << ";execution=false"; message.data = out.str(); lock_status_publisher_->publish(message);
  }

  rclcpp::Node::SharedPtr node_;
  std::string success_topic_, status_topic_, duration_topic_, stage_status_topic_;
  std::string stage_success_topic_, stage_duration_topic_, trajectory_status_topic_;
  std::string plan_lock_status_topic_;
  std::string plan_phase_;
  bool publish_diagnostics_, publish_trajectories_, lock_after_successful_sequence_;
  bool require_dynamic_target_scene_ready_, dynamic_scene_ready_{false};
  aap::GraspClearanceConfig grasp_clearance_config_;
  std::string planning_group_, end_effector_link_;
  int num_planning_attempts_;
  double planning_time_sec_;
  std::string start_state_mode_;
  bool require_fresh_current_start_state_;
  double current_start_state_freshness_sec_;
  std::optional<sensor_msgs::msg::JointState> latest_joint_state_;
  rclcpp::Time latest_joint_state_time_;
  aap::StagePlanningProfile default_profile_, linear_profile_;
  aap::SnapshotLimits snapshot_limits_;
  aap::LinearPathLimits linear_limits_;
  aap::GraspClearanceMetrics last_clearance_metrics_;
  std::vector<std::string> stage_names_;
  std::string stage_sequence_;
  std::unordered_set<std::string> linear_stage_names_;
  std::vector<std::unique_ptr<Stage>> stages_;
  State state_{State::WAITING_FOR_INPUTS};
  std::uint64_t next_plan_id_{0}, locked_plan_id_{0};
  std::int64_t snapshot_stamp_ns_{0};
  std::chrono::steady_clock::time_point last_ignored_status_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr success_publisher_, stage_success_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_, stage_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trajectory_status_publisher_,
    lock_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr duration_publisher_,
    stage_duration_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dynamic_scene_ready_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::TimerBase::SharedPtr scene_ready_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "assembly_sequence_planning_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  try {
    auto planner = std::make_shared<AssemblySequencePlanningNode>(node);
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
