#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

namespace adaptive_assembly_planning
{

struct LinearPathLimits
{
  double max_lateral_deviation{0.002};
  double max_orientation_deviation{0.01};
  double max_endpoint_position_error{0.002};
  double max_endpoint_orientation_error{0.01};
  double max_path_length_ratio{1.02};
  double progress_tolerance{1.0e-4};
  double overshoot_tolerance{1.0e-3};
};

struct LinearPathMetrics
{
  bool valid{false};
  std::string reason;
  double direct_distance{0.0};
  double sampled_cartesian_path_length{0.0};
  double path_length_ratio{std::numeric_limits<double>::infinity()};
  double max_lateral_deviation{0.0};
  double max_orientation_deviation{0.0};
  double endpoint_position_error{0.0};
  double endpoint_orientation_error{0.0};
  double minimum_progress{std::numeric_limits<double>::infinity()};
  double maximum_progress{-std::numeric_limits<double>::infinity()};
  bool monotonic_progress{true};
};

inline bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

inline double quaternion_norm(const geometry_msgs::msg::Quaternion & q)
{
  return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

inline double orientation_distance(
  const geometry_msgs::msg::Quaternion & a,
  const geometry_msgs::msg::Quaternion & b)
{
  const double an = quaternion_norm(a);
  const double bn = quaternion_norm(b);
  if (!(an > 1.0e-12) || !(bn > 1.0e-12)) {
    return std::numeric_limits<double>::infinity();
  }
  const double dot = std::abs(
    (a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w) / (an * bn));
  return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

inline double position_distance(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

inline LinearPathMetrics validate_linear_path(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  const std::vector<geometry_msgs::msg::Pose> & samples,
  const LinearPathLimits & limits)
{
  LinearPathMetrics metrics;
  if (!finite_pose(start) || !finite_pose(goal) || samples.empty()) {
    metrics.reason = "linear_path_invalid_fk";
    return metrics;
  }
  const double dx = goal.position.x - start.position.x;
  const double dy = goal.position.y - start.position.y;
  const double dz = goal.position.z - start.position.z;
  const double squared_distance = dx * dx + dy * dy + dz * dz;
  metrics.direct_distance = std::sqrt(squared_distance);
  if (!(metrics.direct_distance > 1.0e-9)) {
    metrics.reason = "linear_path_zero_length";
    return metrics;
  }

  double previous_progress = -std::numeric_limits<double>::infinity();
  geometry_msgs::msg::Point previous_position = start.position;
  for (const auto & sample : samples) {
    if (!finite_pose(sample) || !(quaternion_norm(sample.orientation) > 1.0e-12)) {
      metrics.reason = "linear_path_invalid_fk";
      return metrics;
    }
    const double px = sample.position.x - start.position.x;
    const double py = sample.position.y - start.position.y;
    const double pz = sample.position.z - start.position.z;
    const double progress = (px * dx + py * dy + pz * dz) / squared_distance;
    const double closest_x = start.position.x + progress * dx;
    const double closest_y = start.position.y + progress * dy;
    const double closest_z = start.position.z + progress * dz;
    const double lateral = std::hypot(
      std::hypot(sample.position.x - closest_x, sample.position.y - closest_y),
      sample.position.z - closest_z);
    metrics.max_lateral_deviation = std::max(metrics.max_lateral_deviation, lateral);
    metrics.max_orientation_deviation = std::max(
      metrics.max_orientation_deviation,
      orientation_distance(start.orientation, sample.orientation));
    metrics.sampled_cartesian_path_length += position_distance(previous_position, sample.position);
    previous_position = sample.position;
    metrics.minimum_progress = std::min(metrics.minimum_progress, progress);
    metrics.maximum_progress = std::max(metrics.maximum_progress, progress);
    if (progress + limits.progress_tolerance < previous_progress) {
      metrics.monotonic_progress = false;
    }
    previous_progress = progress;
  }
  metrics.path_length_ratio = metrics.sampled_cartesian_path_length / metrics.direct_distance;
  metrics.endpoint_position_error = position_distance(samples.back().position, goal.position);
  metrics.endpoint_orientation_error = orientation_distance(
    samples.back().orientation, goal.orientation);

  if (metrics.max_lateral_deviation > limits.max_lateral_deviation) {
    metrics.reason = "linear_path_lateral_deviation";
  } else if (metrics.max_orientation_deviation > limits.max_orientation_deviation) {
    metrics.reason = "linear_path_orientation_deviation";
  } else if (metrics.endpoint_position_error > limits.max_endpoint_position_error ||
    metrics.endpoint_orientation_error > limits.max_endpoint_orientation_error)
  {
    metrics.reason = "linear_path_endpoint_error";
  } else if (!metrics.monotonic_progress) {
    metrics.reason = "linear_path_non_monotonic";
  } else if (metrics.minimum_progress < -limits.overshoot_tolerance ||
    metrics.maximum_progress > 1.0 + limits.overshoot_tolerance)
  {
    metrics.reason = "linear_path_overshoot";
  } else if (metrics.path_length_ratio > limits.max_path_length_ratio) {
    metrics.reason = "linear_path_length_ratio";
  } else {
    metrics.valid = true;
  }
  return metrics;
}

}  // namespace adaptive_assembly_planning
