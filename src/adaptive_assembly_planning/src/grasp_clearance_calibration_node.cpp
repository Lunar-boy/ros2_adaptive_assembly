#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <memory>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "adaptive_assembly_planning/grasp_clearance_validation.hpp"
#include "adaptive_assembly_planning/target_scene_contract.hpp"

namespace aap = adaptive_assembly_planning;

class GraspClearanceCalibration
{
public:
  explicit GraspClearanceCalibration(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
  }

  int run()
  {
    robot_model_loader::RobotModelLoader loader(node_, "robot_description");
    const auto model = loader.getModel();
    if (!model) {return fail("robot_model_unavailable");}
    const auto * group = model->getJointModelGroup("panda_arm");
    if (!group || !model->hasLinkModel("assembly_tcp")) {return fail("panda_model_invalid");}
    auto scene = std::make_shared<planning_scene::PlanningScene>(model);
    moveit::core::RobotState seed(model);
    seed.setToDefaultValues();
    constexpr std::array<const char *, 7> arm_names = {
      "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
      "panda_joint5", "panda_joint6", "panda_joint7"};
    constexpr std::array<double, 7> arm_positions = {
      0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
    for (std::size_t index = 0; index < arm_names.size(); ++index) {
      seed.setVariablePosition(arm_names[index], arm_positions[index]);
    }
    seed.setVariablePosition("panda_finger_joint1", 0.040);
    seed.setVariablePosition("panda_finger_joint2", 0.040);
    seed.update();
    scene->setCurrentState(seed);

    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = model->getModelFrame();
    target.pose.position.x = parameter("target_x", 0.350);
    target.pose.position.y = parameter("target_y", 0.180);
    target.pose.position.z = parameter("target_z", 0.100);
    target.pose.orientation.w = 1.0;
    const double radius = parameter("target_radius", 0.035);
    const double height = parameter("target_height", 0.10);
    if (!scene->processCollisionObjectMsg(
        aap::make_target_cylinder("target_object", model->getModelFrame(), radius, height, target)))
    {return fail("target_apply_failed");}
    auto & acm = scene->getAllowedCollisionMatrixNonConst();
    for (const auto * link : model->getLinkModelsWithCollisionGeometry()) {
      acm.setEntry("target_object", link->getName(), false);
    }
    acm.setEntry("target_object", "panda_leftfinger", true);
    acm.setEntry("target_object", "panda_rightfinger", true);

    aap::GraspClearanceConfig config;
    config.required = true;
    config.minimum_disallowed_clearance = parameter("minimum_clearance", 0.005);
    const double minimum_offset = parameter("minimum_offset", 0.005);
    const double maximum_offset = parameter("maximum_offset", 0.030);
    const double step = parameter("offset_step", 0.001);
    if (!(minimum_offset > 0.0) || maximum_offset < minimum_offset || !(step > 0.0)) {
      return fail("invalid_sweep");
    }

    double selected = std::numeric_limits<double>::quiet_NaN();
    for (double offset = minimum_offset; offset <= maximum_offset + 1.0e-12; offset += step) {
      moveit::core::RobotState state(seed);
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translation() = Eigen::Vector3d(
        target.pose.position.x, target.pose.position.y, target.pose.position.z + offset);
      pose.linear() = Eigen::AngleAxisd(
        std::acos(-1.0), Eigen::Vector3d::UnitX()).toRotationMatrix();
      const bool ik = state.setFromIK(group, pose, "assembly_tcp", 0.25);
      if (!ik) {
        emit_candidate(offset, false, "ik_failed", {});
        continue;
      }
      state.setVariablePosition("panda_finger_joint1", 0.040);
      state.setVariablePosition("panda_finger_joint2", 0.040);
      state.update();
      moveit_msgs::msg::RobotState start;
      moveit::core::robotStateToRobotStateMsg(seed, start, true);
      moveit_msgs::msg::RobotTrajectory trajectory;
      trajectory.joint_trajectory.joint_names = group->getVariableNames();
      trajectory_msgs::msg::JointTrajectoryPoint point;
      state.copyJointGroupPositions(group, point.positions);
      trajectory.joint_trajectory.points.push_back(point);
      auto metrics = aap::evaluate_grasp_clearance(
        scene, model, start, trajectory, config, "assembly_tcp");
      const bool state_valid = !scene->isStateColliding(state, "", false);
      if (!state_valid) {
        metrics.valid = false;
        metrics.grasp_clearance_valid = false;
        metrics.reason = "grasp_clearance_invalid_state";
      }
      emit_candidate(offset, state_valid, metrics.reason, metrics);
      if (metrics.valid && !std::isfinite(selected)) {selected = offset;}
    }
    if (!std::isfinite(selected)) {return fail("no_valid_candidate");}
    RCLCPP_INFO(
      node_->get_logger(), "event=selected;grasp_height_offset=%.3f", selected);
    return 0;
  }

private:
  double parameter(const std::string & name, double default_value)
  {
    if (!node_->has_parameter(name)) {node_->declare_parameter(name, default_value);}
    return node_->get_parameter(name).as_double();
  }

  int fail(const std::string & reason)
  {
    RCLCPP_ERROR(node_->get_logger(), "event=failure;reason=%s", reason.c_str());
    return 1;
  }

  void emit_candidate(
    double offset, bool state_valid, const std::string & reason,
    const aap::GraspClearanceMetrics & metrics)
  {
    std::ostringstream pairs;
    for (std::size_t index = 0; index < metrics.disallowed_collision_pairs.size(); ++index) {
      if (index) {pairs << ',';}
      pairs << metrics.disallowed_collision_pairs[index].first << "<->" <<
        metrics.disallowed_collision_pairs[index].second;
    }
    std::ostringstream target_pairs;
    for (std::size_t index = 0; index < metrics.target_contact_pairs.size(); ++index) {
      if (index) {target_pairs << ',';}
      target_pairs << metrics.target_contact_pairs[index].first << "<->" <<
        metrics.target_contact_pairs[index].second;
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "event=candidate;offset=%.3f;state_valid=%s;valid=%s;"
      "minimum_target_to_robot_distance=%.6f;"
      "minimum_disallowed_clearance=%.6f;"
      "nearest_disallowed_link=%s;disallowed_collision_count=%zu;collision_pairs=%s;"
      "target_contact_pairs=%s;"
      "left_finger_target_geometry_valid=%s;right_finger_target_geometry_valid=%s;reason=%s",
      offset, state_valid ? "true" : "false", metrics.valid ? "true" : "false",
      metrics.minimum_target_to_robot_distance,
      metrics.minimum_disallowed_clearance,
      metrics.nearest_disallowed_link.empty() ? "unavailable" :
      metrics.nearest_disallowed_link.c_str(), metrics.disallowed_collision_count,
      pairs.str().empty() ? "none" : pairs.str().c_str(),
      target_pairs.str().empty() ? "none" : target_pairs.str().c_str(),
      metrics.left_finger_target_geometry_valid ? "true" : "false",
      metrics.right_finger_target_geometry_valid ? "true" : "false",
      reason.empty() ? "none" : reason.c_str());
  }

  rclcpp::Node::SharedPtr node_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "grasp_clearance_calibration_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const int result = GraspClearanceCalibration(node).run();
  rclcpp::shutdown();
  return result;
}
