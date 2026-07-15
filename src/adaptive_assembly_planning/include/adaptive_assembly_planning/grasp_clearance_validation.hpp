#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace adaptive_assembly_planning
{

struct GraspClearanceConfig
{
  bool required{false};
  double minimum_disallowed_clearance{0.005};
  std::string target_object_id{"target_object"};
  std::vector<std::string> allowed_contact_links{"panda_leftfinger", "panda_rightfinger"};
  double synthetic_finger_open_position{0.040};
  double synthetic_finger_closed_position{0.000};
  double synthetic_finger_step{0.001};
};

struct GraspClearanceObservation
{
  bool state_valid{true};
  bool distance_available{true};
  double minimum_target_to_robot_distance{std::numeric_limits<double>::infinity()};
  double minimum_disallowed_clearance{std::numeric_limits<double>::infinity()};
  std::string nearest_disallowed_link;
  std::vector<std::pair<std::string, std::string>> disallowed_collision_pairs;
};

struct GraspFingerGeometryObservation
{
  bool available{true};
  bool left_finger_target_geometry_valid{false};
  bool right_finger_target_geometry_valid{false};
  std::size_t disallowed_collision_count{0};
  std::vector<std::pair<std::string, std::string>> target_contact_pairs;
};

struct GraspClearanceMetrics
{
  bool valid{false};
  std::string reason;
  double grasp_height_offset{std::numeric_limits<double>::quiet_NaN()};
  double minimum_target_to_robot_distance{std::numeric_limits<double>::infinity()};
  double minimum_disallowed_clearance{std::numeric_limits<double>::infinity()};
  std::string nearest_disallowed_link;
  std::size_t disallowed_collision_count{0};
  bool left_finger_target_geometry_valid{false};
  bool right_finger_target_geometry_valid{false};
  bool grasp_clearance_valid{false};
  std::vector<std::pair<std::string, std::string>> disallowed_collision_pairs;
  std::vector<std::pair<std::string, std::string>> target_contact_pairs;
};

bool exact_grasp_finger_allowlist(const std::vector<std::string> & links);

std::string grasp_clearance_context_error(
  bool scene_available, bool target_available, bool robot_model_available,
  bool acm_valid);

GraspClearanceMetrics validate_grasp_clearance_observations(
  const std::vector<GraspClearanceObservation> & observations,
  const GraspFingerGeometryObservation & finger_geometry,
  double minimum_clearance);

GraspClearanceMetrics evaluate_grasp_clearance(
  const planning_scene::PlanningSceneConstPtr & scene,
  const moveit::core::RobotModelConstPtr & robot_model,
  const moveit_msgs::msg::RobotState & start_state,
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const GraspClearanceConfig & config,
  const std::string & end_effector_link);

}  // namespace adaptive_assembly_planning
