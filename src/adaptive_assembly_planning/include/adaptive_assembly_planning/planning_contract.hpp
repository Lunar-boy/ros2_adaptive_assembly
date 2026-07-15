#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "adaptive_assembly_planning/linear_path_validation.hpp"

namespace adaptive_assembly_planning
{

struct StagePlanningProfile
{
  std::string planning_pipeline_id;
  std::string planner_id;
  double position_tolerance;
  double orientation_tolerance;
  double max_velocity_scaling_factor;
  double max_acceleration_scaling_factor;
  bool require_linear_validation;
};

inline StagePlanningProfile resolve_stage_profile(
  const std::string & stage, const std::unordered_set<std::string> & linear_stages,
  const StagePlanningProfile & default_profile, const StagePlanningProfile & linear_profile)
{
  return linear_stages.count(stage) ? linear_profile : default_profile;
}

struct SnapshotLimits
{
  double max_stamp_skew_sec{0.20};
  double approach_min_distance{0.05};
  double approach_max_distance{0.30};
  double approach_max_lateral_offset{0.002};
  double approach_max_orientation_difference{0.01};
  double quaternion_norm_tolerance{1.0e-3};
};

inline double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

inline std::string validate_snapshot(
  const std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> & poses,
  const SnapshotLimits & limits)
{
  const geometry_msgs::msg::PoseStamped * pre_grasp = nullptr;
  const geometry_msgs::msg::PoseStamped * grasp = nullptr;
  double minimum_stamp = std::numeric_limits<double>::infinity();
  double maximum_stamp = -std::numeric_limits<double>::infinity();
  bool have_nonzero_stamp = false;
  for (const auto & named_pose : poses) {
    const auto & pose = named_pose.second;
    if (pose.header.frame_id.empty()) {
      return "snapshot_empty_frame";
    }
    if (!finite_pose(pose.pose)) {
      return "snapshot_non_finite_pose";
    }
    const double norm = quaternion_norm(pose.pose.orientation);
    if (!(norm > 1.0e-12) || std::abs(norm - 1.0) > limits.quaternion_norm_tolerance) {
      return "snapshot_invalid_quaternion";
    }
    const double stamp = stamp_seconds(pose.header.stamp);
    if (stamp > 0.0) {
      have_nonzero_stamp = true;
      minimum_stamp = std::min(minimum_stamp, stamp);
      maximum_stamp = std::max(maximum_stamp, stamp);
    }
    if (named_pose.first == "pre_grasp") {
      pre_grasp = &pose;
    } else if (named_pose.first == "grasp") {
      grasp = &pose;
    }
  }
  if (have_nonzero_stamp && maximum_stamp - minimum_stamp > limits.max_stamp_skew_sec) {
    return "snapshot_stamp_skew";
  }
  if (pre_grasp && grasp) {
    if (pre_grasp->header.frame_id != grasp->header.frame_id) {
      return "linear_approach_frame_mismatch";
    }
    const double orientation_error = orientation_distance(
      pre_grasp->pose.orientation, grasp->pose.orientation);
    if (orientation_error > limits.approach_max_orientation_difference) {
      return "linear_approach_orientation_mismatch";
    }
    const double distance = position_distance(pre_grasp->pose.position, grasp->pose.position);
    if (distance < limits.approach_min_distance) {
      return "linear_approach_too_short";
    }
    if (distance > limits.approach_max_distance) {
      return "linear_approach_too_long";
    }
    const double lateral = std::hypot(
      pre_grasp->pose.position.x - grasp->pose.position.x,
      pre_grasp->pose.position.y - grasp->pose.position.y);
    if (lateral > limits.approach_max_lateral_offset) {
      return "linear_approach_lateral_offset";
    }
  }
  return "";
}

}  // namespace adaptive_assembly_planning
