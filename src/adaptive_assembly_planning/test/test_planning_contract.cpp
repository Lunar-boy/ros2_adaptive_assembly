#include <cmath>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "adaptive_assembly_planning/linear_path_validation.hpp"
#include "adaptive_assembly_planning/planning_contract.hpp"
#include "adaptive_assembly_planning/target_scene_contract.hpp"

namespace aap = adaptive_assembly_planning;

geometry_msgs::msg::Pose pose(double x, double y, double z, double angle = 0.0)
{
  geometry_msgs::msg::Pose result;
  result.position.x = x; result.position.y = y; result.position.z = z;
  result.orientation.z = std::sin(angle / 2.0);
  result.orientation.w = std::cos(angle / 2.0);
  return result;
}

geometry_msgs::msg::PoseStamped stamped(
  const std::string & frame, double x, double y, double z, double stamp_sec = 1.0,
  double angle = 0.0)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = frame;
  result.header.stamp.sec = static_cast<std::int32_t>(stamp_sec);
  result.header.stamp.nanosec = static_cast<std::uint32_t>(
    (stamp_sec - std::floor(stamp_sec)) * 1.0e9);
  result.pose = pose(x, y, z, angle);
  return result;
}

TEST(StageProfiles, LinearDoesNotLeak)
{
  const aap::StagePlanningProfile defaults{"ompl", "", 0.005, 0.03, 1.0, 1.0, false};
  const aap::StagePlanningProfile linear{"pilz_industrial_motion_planner", "LIN", 0.002, 0.01, 0.05,
    0.05, true};
  const std::unordered_set<std::string> stages{"grasp"};
  EXPECT_EQ(aap::resolve_stage_profile("grasp", stages, defaults, linear).planner_id, "LIN");
  for (const auto & stage : {"pre_grasp", "lift", "pre_place", "place", "retreat"}) {
    const auto profile = aap::resolve_stage_profile(stage, stages, defaults, linear);
    EXPECT_EQ(profile.planning_pipeline_id, "ompl");
    EXPECT_FALSE(profile.require_linear_validation);
    EXPECT_DOUBLE_EQ(profile.max_velocity_scaling_factor, 1.0);
  }
}

TEST(Snapshot, AcceptsPhysicalApproachAndRejectsBoundaries)
{
  aap::SnapshotLimits limits;
  auto pre = stamped("panda_link0", 0.44, 0.15, 0.35, 1.0);
  auto grasp = stamped("panda_link0", 0.44, 0.15, 0.15, 1.1);
  std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> values{
    {"pre_grasp", pre}, {"grasp", grasp}};
  EXPECT_EQ(aap::validate_snapshot(values, limits), "");
  values[1].second.pose.position.x += 0.003;
  EXPECT_EQ(aap::validate_snapshot(values, limits), "linear_approach_lateral_offset");
  values[1].second = grasp; values[1].second.header.frame_id = "world";
  EXPECT_EQ(aap::validate_snapshot(values, limits), "linear_approach_frame_mismatch");
  values[1].second = grasp; values[1].second.header.stamp.sec = 2;
  EXPECT_EQ(aap::validate_snapshot(values, limits), "snapshot_stamp_skew");
  values[1].second = grasp; values[1].second.pose.position.z = 0.31;
  EXPECT_EQ(aap::validate_snapshot(values, limits), "linear_approach_too_short");
  values[1].second = grasp; values[1].second.pose.position.z = 0.0;
  EXPECT_EQ(aap::validate_snapshot(values, limits), "linear_approach_too_long");
  values[1].second = grasp; values[1].second.pose.orientation.w = 0.0;
  EXPECT_EQ(aap::validate_snapshot(values, limits), "snapshot_invalid_quaternion");
  values[1].second = grasp; values[1].second.pose.position.x =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(aap::validate_snapshot(values, limits), "snapshot_non_finite_pose");
  values[1].second = stamped("panda_link0", 0.44, 0.15, 0.15, 1.1, 0.02);
  EXPECT_EQ(aap::validate_snapshot(values, limits), "linear_approach_orientation_mismatch");
}

TEST(LinearPath, ExactAndNumericDeviation)
{
  const auto start = pose(0.0, 0.0, 0.2);
  const auto goal = pose(0.0, 0.0, 0.0);
  auto exact = aap::validate_linear_path(
    start, goal, {pose(0.0, 0.0, 0.2), pose(0.0, 0.0, 0.1), goal}, {});
  EXPECT_TRUE(exact.valid); EXPECT_NEAR(exact.path_length_ratio, 1.0, 1e-12);
  auto numeric = aap::validate_linear_path(
    start, goal, {pose(0.0, 0.0, 0.2), pose(0.001, 0.0, 0.1), goal}, {});
  EXPECT_TRUE(numeric.valid); EXPECT_LE(numeric.max_lateral_deviation, 0.002);
}

TEST(LinearPath, RejectsLateralCurvedEndpointAndOrientation)
{
  const auto start = pose(0.0, 0.0, 0.2);
  const auto goal = pose(0.0, 0.0, 0.0);
  EXPECT_EQ(aap::validate_linear_path(
      start, goal, {start, pose(0.003, 0.0, 0.1), goal}, {}).reason,
    "linear_path_lateral_deviation");
  std::vector<geometry_msgs::msg::Pose> curved{start};
  for (int index = 1; index < 20; ++index) {
    curved.push_back(pose(index % 2 ? 0.0019 : -0.0019, 0.0, 0.2 - index * 0.01));
  }
  curved.push_back(goal);
  EXPECT_EQ(aap::validate_linear_path(start, goal, curved, {}).reason,
    "linear_path_length_ratio");
  EXPECT_EQ(aap::validate_linear_path(
      start, goal, {start, pose(0.0, 0.0, 0.1), pose(0.0, 0.0, 0.003)}, {}).reason,
    "linear_path_endpoint_error");
  EXPECT_EQ(aap::validate_linear_path(
      start, goal, {start, pose(0.0, 0.0, 0.1, 0.02), goal}, {}).reason,
    "linear_path_orientation_deviation");
}

TEST(LinearPath, RejectsProgressOvershootZeroLengthAndMalformed)
{
  const auto start = pose(0.0, 0.0, 0.2);
  const auto goal = pose(0.0, 0.0, 0.0);
  EXPECT_EQ(aap::validate_linear_path(
      start, goal, {start, pose(0.0, 0.0, 0.08), pose(0.0, 0.0, 0.1), goal}, {}).reason,
    "linear_path_non_monotonic");
  EXPECT_EQ(aap::validate_linear_path(
      start, goal, {pose(0.0, 0.0, 0.201), pose(0.0, 0.0, 0.1), goal}, {}).reason,
    "linear_path_overshoot");
  EXPECT_EQ(aap::validate_linear_path(start, start, {start}, {}).reason,
    "linear_path_zero_length");
  auto malformed = start; malformed.position.x = std::numeric_limits<double>::infinity();
  EXPECT_EQ(aap::validate_linear_path(start, goal, {malformed}, {}).reason,
    "linear_path_invalid_fk");
}

TEST(TargetScene, CylinderAndExactFingerAcm)
{
  const auto target_pose = stamped("panda_link0", 0.44, 0.15, 0.15);
  const auto object = aap::make_target_cylinder(
    "target_object", "panda_link0", 0.035, 0.10, target_pose);
  ASSERT_EQ(object.primitives.size(), 1U);
  EXPECT_EQ(object.primitives[0].type, shape_msgs::msg::SolidPrimitive::CYLINDER);
  EXPECT_DOUBLE_EQ(
    object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT],
    0.10);
  EXPECT_DOUBLE_EQ(
    object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS],
    0.035);

  moveit_msgs::msg::AllowedCollisionMatrix acm;
  for (const auto & name : {"panda_hand", "panda_link7", "panda_link8", "assembly_tcp"}) {
    aap::ensure_acm_name(acm, name);
  }
  aap::configure_target_acm(
    acm, "target_object", {"panda_leftfinger", "panda_rightfinger"});
  aap::set_acm_pair(acm, "panda_link0", "work_table", true);
  const auto target = static_cast<std::size_t>(std::distance(
      acm.entry_names.begin(),
      std::find(acm.entry_names.begin(), acm.entry_names.end(), "target_object")));
  for (std::size_t index = 0; index < acm.entry_names.size(); ++index) {
    const bool expected = acm.entry_names[index] == "panda_leftfinger" ||
      acm.entry_names[index] == "panda_rightfinger";
    if (index != target) {
      EXPECT_EQ(acm.entry_values[target].enabled[index], expected) << acm.entry_names[index];
    }
  }
  const auto base = static_cast<std::size_t>(std::distance(
      acm.entry_names.begin(),
      std::find(acm.entry_names.begin(), acm.entry_names.end(), "panda_link0")));
  const auto table = static_cast<std::size_t>(std::distance(
      acm.entry_names.begin(),
      std::find(acm.entry_names.begin(), acm.entry_names.end(), "work_table")));
  EXPECT_TRUE(acm.entry_values[base].enabled[table]);
}
