#pragma once

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace adaptive_assembly_planning
{

inline void ensure_acm_name(
  moveit_msgs::msg::AllowedCollisionMatrix & acm, const std::string & name)
{
  if (std::find(acm.entry_names.begin(), acm.entry_names.end(), name) !=
    acm.entry_names.end())
  {
    return;
  }
  const std::size_t old_size = acm.entry_names.size();
  acm.entry_names.push_back(name);
  for (auto & row : acm.entry_values) {
    row.enabled.resize(old_size + 1, false);
  }
  moveit_msgs::msg::AllowedCollisionEntry entry;
  entry.enabled.resize(old_size + 1, false);
  acm.entry_values.push_back(entry);
}

inline void set_acm_pair(
  moveit_msgs::msg::AllowedCollisionMatrix & acm,
  const std::string & first, const std::string & second, bool allowed)
{
  ensure_acm_name(acm, first);
  ensure_acm_name(acm, second);
  const auto a = static_cast<std::size_t>(std::distance(
      acm.entry_names.begin(),
      std::find(acm.entry_names.begin(), acm.entry_names.end(), first)));
  const auto b = static_cast<std::size_t>(std::distance(
      acm.entry_names.begin(),
      std::find(acm.entry_names.begin(), acm.entry_names.end(), second)));
  acm.entry_values[a].enabled[b] = allowed;
  acm.entry_values[b].enabled[a] = allowed;
}

inline void configure_target_acm(
  moveit_msgs::msg::AllowedCollisionMatrix & acm,
  const std::string & object_id, const std::vector<std::string> & allowed_links)
{
  ensure_acm_name(acm, object_id);
  const auto existing_names = acm.entry_names;
  for (const auto & name : existing_names) {
    if (name != object_id) {
      set_acm_pair(acm, object_id, name, false);
    }
  }
  for (const auto & link : allowed_links) {
    set_acm_pair(acm, object_id, link, true);
  }
}

inline moveit_msgs::msg::CollisionObject make_target_cylinder(
  const std::string & object_id, const std::string & planning_frame,
  double radius, double height, const geometry_msgs::msg::PoseStamped & pose)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = planning_frame;
  object.id = object_id;
  shape_msgs::msg::SolidPrimitive cylinder;
  cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  cylinder.dimensions.resize(2);
  cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = height;
  cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = radius;
  object.primitives.push_back(cylinder);
  object.primitive_poses.push_back(pose.pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

}  // namespace adaptive_assembly_planning
