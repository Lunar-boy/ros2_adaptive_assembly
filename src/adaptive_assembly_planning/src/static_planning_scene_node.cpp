#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

class StaticPlanningSceneNode
{
public:
  explicit StaticPlanningSceneNode(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    planning_frame_(declare_parameter<std::string>("planning_frame", "panda_link0")),
    apply_delay_sec_(declare_parameter<double>("apply_delay_sec", 1.0)),
    add_work_table_(declare_parameter<bool>("add_work_table", true)),
    table_x_(declare_parameter<double>("table_x", 0.45)),
    table_y_(declare_parameter<double>("table_y", 0.0)),
    table_z_(declare_parameter<double>("table_z", -0.04)),
    table_size_x_(declare_parameter<double>("table_size_x", 0.80)),
    table_size_y_(declare_parameter<double>("table_size_y", 0.80)),
    table_size_z_(declare_parameter<double>("table_size_z", 0.04)),
    add_target_support_(declare_parameter<bool>("add_target_support", true)),
    target_support_x_(declare_parameter<double>("target_support_x", 0.45)),
    target_support_y_(declare_parameter<double>("target_support_y", 0.0)),
    target_support_z_(declare_parameter<double>("target_support_z", 0.01)),
    target_support_size_x_(declare_parameter<double>("target_support_size_x", 0.12)),
    target_support_size_y_(declare_parameter<double>("target_support_size_y", 0.12)),
    target_support_size_z_(declare_parameter<double>("target_support_size_z", 0.02))
  {
    ready_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/planning_scene_objects_ready",
      rclcpp::QoS(1).transient_local().reliable());

    const auto delay = std::chrono::duration<double>(
      std::max(0.0, apply_delay_sec_));
    apply_timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(delay),
      [this]() {
        apply_static_objects();
      });

    RCLCPP_INFO(
      node_->get_logger(),
      "Static PlanningScene node ready for frame '%s'. Static collision "
      "objects only; trajectory execution is disabled in this PR.",
      planning_frame_.c_str());
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

  void apply_static_objects()
  {
    apply_timer_->cancel();

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    if (add_work_table_) {
      collision_objects.push_back(make_box_collision_object(
        "work_table",
        table_x_, table_y_, table_z_,
        table_size_x_, table_size_y_, table_size_z_));
    }
    if (add_target_support_) {
      collision_objects.push_back(make_box_collision_object(
        "target_support",
        target_support_x_, target_support_y_, target_support_z_,
        target_support_size_x_, target_support_size_y_, target_support_size_z_));
    }

    if (collision_objects.empty()) {
      RCLCPP_WARN(
        node_->get_logger(),
        "No static PlanningScene collision objects are enabled.");
      publish_ready(false);
      return;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Applying %zu static PlanningScene collision object(s) in frame '%s'.",
      collision_objects.size(), planning_frame_.c_str());

    const bool applied = planning_scene_interface_.applyCollisionObjects(
      collision_objects);
    publish_ready(applied);

    if (applied) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Static PlanningScene collision objects applied. This node does not "
        "execute motion.");
    } else {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to apply static PlanningScene collision objects.");
    }
  }

  moveit_msgs::msg::CollisionObject make_box_collision_object(
    const std::string & object_id,
    const double x,
    const double y,
    const double z,
    const double size_x,
    const double size_y,
    const double size_z) const
  {
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {size_x, size_y, size_z};

    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = planning_frame_;
    collision_object.id = object_id;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    RCLCPP_INFO(
      node_->get_logger(),
      "Prepared collision object '%s': size=(%.3f, %.3f, %.3f), "
      "pose=(x=%.3f, y=%.3f, z=%.3f, qx=0.000, qy=0.000, qz=0.000, qw=1.000)",
      object_id.c_str(), size_x, size_y, size_z, x, y, z);

    return collision_object;
  }

  void publish_ready(const bool ready)
  {
    std_msgs::msg::Bool message;
    message.data = ready;
    ready_publisher_->publish(message);
  }

  rclcpp::Node::SharedPtr node_;
  std::string planning_frame_;
  double apply_delay_sec_;
  bool add_work_table_;
  double table_x_;
  double table_y_;
  double table_z_;
  double table_size_x_;
  double table_size_y_;
  double table_size_z_;
  bool add_target_support_;
  double target_support_x_;
  double target_support_y_;
  double target_support_z_;
  double target_support_size_x_;
  double target_support_size_y_;
  double target_support_size_z_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
  rclcpp::TimerBase::SharedPtr apply_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "static_planning_scene_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto static_planning_scene = std::make_shared<StaticPlanningSceneNode>(node);

  rclcpp::spin(node);
  static_planning_scene.reset();
  rclcpp::shutdown();
  return 0;
}
