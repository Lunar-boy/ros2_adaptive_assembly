#include "adaptive_assembly_planning/grasp_clearance_validation.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_set>

#include <geometric_shapes/shapes.h>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/collision_detection/collision_matrix.hpp>
#include <moveit/robot_state/conversions.hpp>

namespace adaptive_assembly_planning
{
namespace
{

bool is_target_pair(
  const std::pair<std::string, std::string> & pair, const std::string & target)
{
  return pair.first == target || pair.second == target;
}

std::string other_name(
  const std::pair<std::string, std::string> & pair, const std::string & target)
{
  if (pair.first == target) {return pair.second;}
  if (pair.second == target) {return pair.first;}
  return "";
}

collision_detection::AllowedCollisionMatrix target_isolation_acm(
  const planning_scene::PlanningSceneConstPtr & scene,
  const moveit::core::RobotModelConstPtr & robot_model,
  const GraspClearanceConfig & config, bool include_fingers)
{
  std::vector<std::string> names;
  for (const auto * link : robot_model->getLinkModelsWithCollisionGeometry()) {
    names.push_back(link->getName());
  }
  const auto world_names = scene->getWorld()->getObjectIds();
  names.insert(names.end(), world_names.begin(), world_names.end());
  collision_detection::AllowedCollisionMatrix acm(names, true);
  const std::unordered_set<std::string> allowed(
    config.allowed_contact_links.begin(), config.allowed_contact_links.end());
  for (const auto * link : robot_model->getLinkModelsWithCollisionGeometry()) {
    const bool finger = allowed.count(link->getName()) > 0;
    acm.setEntry(config.target_object_id, link->getName(), !include_fingers && finger);
  }
  return acm;
}

bool exact_finger_only_acm(
  const planning_scene::PlanningSceneConstPtr & scene,
  const moveit::core::RobotModelConstPtr & robot_model,
  const GraspClearanceConfig & config)
{
  if (config.allowed_contact_links.size() != 2 ||
    config.allowed_contact_links[0] != "panda_leftfinger" ||
    config.allowed_contact_links[1] != "panda_rightfinger")
  {return false;}
  const std::unordered_set<std::string> allowed(
    config.allowed_contact_links.begin(), config.allowed_contact_links.end());
  const auto & acm = scene->getAllowedCollisionMatrix();
  for (const auto * link : robot_model->getLinkModelsWithCollisionGeometry()) {
    collision_detection::AllowedCollision::Type type;
    const bool entry = acm.getAllowedCollision(config.target_object_id, link->getName(), type);
    const bool is_allowed = entry && type == collision_detection::AllowedCollision::ALWAYS;
    if (is_allowed != (allowed.count(link->getName()) > 0)) {return false;}
  }
  return true;
}

GraspClearanceObservation inspect_state(
  const planning_scene::PlanningSceneConstPtr & scene,
  const moveit::core::RobotModelConstPtr & robot_model,
  const moveit::core::RobotState & state,
  const GraspClearanceConfig & config)
{
  GraspClearanceObservation observation;
  auto isolation_acm = target_isolation_acm(scene, robot_model, config, false);
  auto all_target_acm = target_isolation_acm(scene, robot_model, config, true);
  collision_detection::CollisionRequest collision_request;
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;
  collision_request.max_contacts_per_pair = 100;
  collision_detection::CollisionResult collision_result;
  scene->checkCollision(collision_request, collision_result, state, isolation_acm);
  for (const auto & [pair, contacts] : collision_result.contacts) {
    if (is_target_pair(pair, config.target_object_id) && !contacts.empty()) {
      observation.disallowed_collision_pairs.push_back(pair);
    }
  }

  collision_detection::DistanceRequest distance_request;
  distance_request.type = collision_detection::DistanceRequestTypes::GLOBAL;
  distance_request.enable_signed_distance = true;
  distance_request.enable_nearest_points = true;
  distance_request.acm = &isolation_acm;
  collision_detection::DistanceResult distance_result;
  scene->getCollisionEnv()->distanceRobot(distance_request, distance_result, state);
  const auto & minimum = distance_result.minimum_distance;
  const std::pair<std::string, std::string> pair{
    minimum.link_names[0], minimum.link_names[1]};
  if (!std::isfinite(minimum.distance) || !is_target_pair(pair, config.target_object_id)) {
    observation.distance_available = false;
  } else {
    observation.minimum_disallowed_clearance = minimum.distance;
    observation.nearest_disallowed_link = other_name(pair, config.target_object_id);
    observation.distance_available = !observation.nearest_disallowed_link.empty();
  }
  collision_detection::DistanceRequest all_distance_request;
  all_distance_request.type = collision_detection::DistanceRequestTypes::GLOBAL;
  all_distance_request.enable_signed_distance = true;
  all_distance_request.acm = &all_target_acm;
  collision_detection::DistanceResult all_distance_result;
  scene->getCollisionEnv()->distanceRobot(
    all_distance_request, all_distance_result, state);
  observation.minimum_target_to_robot_distance =
    all_distance_result.minimum_distance.distance;
  if (!std::isfinite(observation.minimum_target_to_robot_distance)) {
    observation.distance_available = false;
  }
  return observation;
}

GraspFingerGeometryObservation inspect_finger_geometry(
  const planning_scene::PlanningSceneConstPtr & scene,
  const moveit::core::RobotModelConstPtr & robot_model,
  const moveit::core::RobotState & final_arm_state,
  const GraspClearanceConfig & config)
{
  GraspFingerGeometryObservation result;
  if (!robot_model->hasJointModel("panda_finger_joint1") ||
    !robot_model->hasJointModel("panda_finger_joint2") ||
    !(config.synthetic_finger_step > 0.0) ||
    config.synthetic_finger_open_position < config.synthetic_finger_closed_position)
  {
    result.available = false;
    return result;
  }
  auto isolation_acm = target_isolation_acm(scene, robot_model, config, true);
  for (double position = config.synthetic_finger_open_position;
    position + 1.0e-12 >= config.synthetic_finger_closed_position;
    position -= config.synthetic_finger_step)
  {
    moveit::core::RobotState state(final_arm_state);
    state.setVariablePosition("panda_finger_joint1", position);
    state.setVariablePosition("panda_finger_joint2", position);
    state.update();
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 1000;
    request.max_contacts_per_pair = 100;
    collision_detection::CollisionResult collision;
    scene->checkCollision(request, collision, state, isolation_acm);
    bool left = false;
    bool right = false;
    std::size_t disallowed = 0;
    std::vector<std::pair<std::string, std::string>> target_pairs;
    for (const auto & [pair, contacts] : collision.contacts) {
      if (!is_target_pair(pair, config.target_object_id) || contacts.empty()) {continue;}
      const std::string link = other_name(pair, config.target_object_id);
      left = left || link == "panda_leftfinger";
      right = right || link == "panda_rightfinger";
      target_pairs.push_back(pair);
      if (std::find(
          config.allowed_contact_links.begin(), config.allowed_contact_links.end(), link) ==
        config.allowed_contact_links.end())
      {++disallowed;}
    }
    if (left && right && disallowed == 0) {
      result.left_finger_target_geometry_valid = true;
      result.right_finger_target_geometry_valid = true;
      result.target_contact_pairs = std::move(target_pairs);
      return result;
    }
    result.target_contact_pairs = std::move(target_pairs);
    result.disallowed_collision_count = std::max(result.disallowed_collision_count, disallowed);
  }
  return result;
}

}  // namespace

bool exact_grasp_finger_allowlist(const std::vector<std::string> & links)
{
  auto sorted = links;
  std::sort(sorted.begin(), sorted.end());
  return sorted == std::vector<std::string>({"panda_leftfinger", "panda_rightfinger"});
}

std::string grasp_clearance_context_error(
  bool scene_available, bool target_available, bool robot_model_available,
  bool acm_valid)
{
  if (!scene_available || !robot_model_available || !acm_valid) {
    return "grasp_clearance_scene_unavailable";
  }
  if (!target_available) {return "grasp_clearance_target_missing";}
  return "";
}

GraspClearanceMetrics validate_grasp_clearance_observations(
  const std::vector<GraspClearanceObservation> & observations,
  const GraspFingerGeometryObservation & finger_geometry,
  double minimum_clearance)
{
  GraspClearanceMetrics metrics;
  if (observations.empty() || !std::isfinite(minimum_clearance) || minimum_clearance < 0.0) {
    metrics.reason = "grasp_clearance_invalid_state";
    return metrics;
  }
  for (const auto & observation : observations) {
    if (!observation.state_valid) {
      metrics.reason = "grasp_clearance_invalid_state";
      return metrics;
    }
    if (!observation.distance_available ||
      !std::isfinite(observation.minimum_disallowed_clearance) ||
      observation.nearest_disallowed_link.empty())
    {
      metrics.reason = "grasp_clearance_distance_unavailable";
      return metrics;
    }
    if (observation.minimum_disallowed_clearance < metrics.minimum_disallowed_clearance) {
      metrics.minimum_disallowed_clearance = observation.minimum_disallowed_clearance;
      metrics.nearest_disallowed_link = observation.nearest_disallowed_link;
    }
    metrics.minimum_target_to_robot_distance = std::min(
      metrics.minimum_target_to_robot_distance,
      observation.minimum_target_to_robot_distance);
    metrics.disallowed_collision_count += observation.disallowed_collision_pairs.size();
    metrics.disallowed_collision_pairs.insert(
      metrics.disallowed_collision_pairs.end(), observation.disallowed_collision_pairs.begin(),
      observation.disallowed_collision_pairs.end());
  }
  metrics.left_finger_target_geometry_valid =
    finger_geometry.left_finger_target_geometry_valid;
  metrics.right_finger_target_geometry_valid =
    finger_geometry.right_finger_target_geometry_valid;
  metrics.disallowed_collision_count += finger_geometry.disallowed_collision_count;
  metrics.target_contact_pairs = finger_geometry.target_contact_pairs;
  if (metrics.disallowed_collision_count > 0) {
    metrics.reason = "grasp_disallowed_collision";
  } else if (metrics.minimum_disallowed_clearance + 1.0e-12 < minimum_clearance) {
    metrics.reason = "grasp_clearance_below_minimum";
  } else if (!finger_geometry.available ||
    !metrics.left_finger_target_geometry_valid ||
    !metrics.right_finger_target_geometry_valid)
  {
    metrics.reason = "grasp_finger_geometry_invalid";
  } else {
    metrics.valid = true;
    metrics.grasp_clearance_valid = true;
  }
  return metrics;
}

GraspClearanceMetrics evaluate_grasp_clearance(
  const planning_scene::PlanningSceneConstPtr & scene,
  const moveit::core::RobotModelConstPtr & robot_model,
  const moveit_msgs::msg::RobotState & start_state,
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const GraspClearanceConfig & config,
  const std::string & end_effector_link)
{
  GraspClearanceMetrics failure;
  const std::string initial_context = grasp_clearance_context_error(
    static_cast<bool>(scene), false, static_cast<bool>(robot_model), true);
  if (!scene || !robot_model) {
    failure.reason = initial_context;
    return failure;
  }
  const auto world = scene->getWorld();
  const auto target = world ? world->getObject(config.target_object_id) : nullptr;
  if (!target || target->shapes_.size() != 1 ||
    target->shapes_[0]->type != shapes::CYLINDER)
  {
    failure.reason = grasp_clearance_context_error(true, false, true, true);
    return failure;
  }
  if (!robot_model->hasLinkModel(end_effector_link) ||
    !exact_grasp_finger_allowlist(config.allowed_contact_links) ||
    !exact_finger_only_acm(scene, robot_model, config))
  {
    failure.reason = "grasp_clearance_scene_unavailable";
    return failure;
  }
  const auto & joint_trajectory = trajectory.joint_trajectory;
  if (joint_trajectory.joint_names.empty() || joint_trajectory.points.empty()) {
    failure.reason = "grasp_clearance_invalid_state";
    return failure;
  }
  moveit::core::RobotState state(robot_model);
  try {
    moveit::core::robotStateMsgToRobotState(start_state, state, true);
  } catch (const std::exception &) {
    failure.reason = "grasp_clearance_invalid_state";
    return failure;
  }
  std::vector<GraspClearanceObservation> observations;
  observations.reserve(joint_trajectory.points.size());
  for (const auto & point : joint_trajectory.points) {
    if (point.positions.size() != joint_trajectory.joint_names.size() ||
      !std::all_of(point.positions.begin(), point.positions.end(), [](double value) {
        return std::isfinite(value);
      }))
    {
      failure.reason = "grasp_clearance_invalid_state";
      return failure;
    }
    try {
      state.setVariablePositions(joint_trajectory.joint_names, point.positions);
      state.update();
    } catch (const std::exception &) {
      failure.reason = "grasp_clearance_invalid_state";
      return failure;
    }
    observations.push_back(inspect_state(scene, robot_model, state, config));
  }
  const Eigen::Isometry3d target_pose = world->getTransform(config.target_object_id);
  const Eigen::Isometry3d tcp_pose = state.getGlobalLinkTransform(end_effector_link);
  const auto finger_geometry = inspect_finger_geometry(scene, robot_model, state, config);
  auto metrics = validate_grasp_clearance_observations(
    observations, finger_geometry, config.minimum_disallowed_clearance);
  metrics.grasp_height_offset = tcp_pose.translation().z() - target_pose.translation().z();
  return metrics;
}

}  // namespace adaptive_assembly_planning
