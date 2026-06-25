#include <cmath>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class DynamicTargetSceneNode
{
public:
  explicit DynamicTargetSceneNode(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    input_pose_topic_(declare_parameter<std::string>(
        "input_pose_topic", "/panda_pre_grasp_pose")),
    object_id_(declare_parameter<std::string>("object_id", "target_object_dynamic")),
    size_x_(declare_parameter<double>("size_x", 0.05)),
    size_y_(declare_parameter<double>("size_y", 0.05)),
    size_z_(declare_parameter<double>("size_z", 0.04)),
    x_offset_(declare_parameter<double>("x_offset", 0.0)),
    y_offset_(declare_parameter<double>("y_offset", 0.0)),
    z_offset_(declare_parameter<double>("z_offset", -0.20)),
    min_update_distance_(declare_parameter<double>("min_update_distance", 0.02)),
    publish_updates_(declare_parameter<bool>("publish_updates", true))
  {
    ready_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/dynamic_target_scene_ready",
      rclcpp::QoS(1).transient_local().reliable());
    status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
      "/dynamic_target_scene_status", 10);
    pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      input_pose_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        pose_callback(*message);
      });

    RCLCPP_INFO(
      node_->get_logger(),
      "Dynamic target scene node ready: input_pose_topic='%s', object_id='%s', "
      "size=(%.3f, %.3f, %.3f), offsets=(%.3f, %.3f, %.3f), "
      "min_update_distance=%.3f, publish_updates=%s. This node updates the "
      "PlanningScene only and does not execute motion.",
      input_pose_topic_.c_str(), object_id_.c_str(), size_x_, size_y_, size_z_,
      x_offset_, y_offset_, z_offset_, min_update_distance_,
      publish_updates_ ? "true" : "false");
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

  void pose_callback(const geometry_msgs::msg::PoseStamped & input_pose)
  {
    const auto & input_position = input_pose.pose.position;
    RCLCPP_INFO(
      node_->get_logger(),
      "Received Panda pre-grasp pose for dynamic target scene: frame='%s', "
      "x=%.3f, y=%.3f, z=%.3f",
      input_pose.header.frame_id.c_str(), input_position.x, input_position.y,
      input_position.z);

    if (input_pose.header.frame_id.empty()) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Skipping dynamic target scene update because the incoming pose frame "
        "is empty.");
      publish_status(
        "skipped_empty_frame", input_pose.header.frame_id, geometry_msgs::msg::Pose(),
        std::nullopt);
      return;
    }

    const auto object_pose = compute_object_pose(input_pose);
    RCLCPP_INFO(
      node_->get_logger(),
      "Computed dynamic target collision pose: frame='%s', x=%.3f, y=%.3f, "
      "z=%.3f, qx=0.000, qy=0.000, qz=0.000, qw=1.000",
      input_pose.header.frame_id.c_str(), object_pose.position.x,
      object_pose.position.y, object_pose.position.z);

    std::optional<double> distance_from_last_update;
    if (last_applied_pose_.has_value()) {
      distance_from_last_update = position_distance(
        object_pose, last_applied_pose_.value());
      if (distance_from_last_update.value() < min_update_distance_) {
        RCLCPP_INFO(
          node_->get_logger(),
          "Skipping dynamic target scene update: object moved %.3f m, below "
          "min_update_distance %.3f m.",
          distance_from_last_update.value(), min_update_distance_);
        publish_status(
          "skipped_small_motion", input_pose.header.frame_id, object_pose,
          distance_from_last_update);
        return;
      }
    }

    const bool applied = planning_scene_interface_.applyCollisionObject(
      make_collision_object(input_pose.header.frame_id, object_pose));

    if (applied) {
      ready_ = true;
      last_applied_pose_ = object_pose;
      publish_ready(true);
      publish_status(
        "updated", input_pose.header.frame_id, object_pose,
        distance_from_last_update);
      RCLCPP_INFO(
        node_->get_logger(),
        "Applied dynamic target collision object '%s'. PlanningScene updated; "
        "trajectory execution remains disabled.",
        object_id_.c_str());
    } else {
      publish_status(
        "failed", input_pose.header.frame_id, object_pose,
        distance_from_last_update);
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to apply dynamic target collision object '%s'.",
        object_id_.c_str());
    }
  }

  geometry_msgs::msg::Pose compute_object_pose(
    const geometry_msgs::msg::PoseStamped & input_pose) const
  {
    geometry_msgs::msg::Pose object_pose;
    object_pose.position.x = input_pose.pose.position.x + x_offset_;
    object_pose.position.y = input_pose.pose.position.y + y_offset_;
    object_pose.position.z = input_pose.pose.position.z + z_offset_;
    object_pose.orientation.x = 0.0;
    object_pose.orientation.y = 0.0;
    object_pose.orientation.z = 0.0;
    object_pose.orientation.w = 1.0;
    return object_pose;
  }

  moveit_msgs::msg::CollisionObject make_collision_object(
    const std::string & frame_id,
    const geometry_msgs::msg::Pose & object_pose) const
  {
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {size_x_, size_y_, size_z_};

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = object_id_;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(object_pose);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
    return collision_object;
  }

  static double position_distance(
    const geometry_msgs::msg::Pose & first,
    const geometry_msgs::msg::Pose & second)
  {
    const double dx = first.position.x - second.position.x;
    const double dy = first.position.y - second.position.y;
    const double dz = first.position.z - second.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  void publish_ready(const bool ready)
  {
    std_msgs::msg::Bool message;
    message.data = ready;
    ready_publisher_->publish(message);
  }

  void publish_status(
    const std::string & event,
    const std::string & frame_id,
    const geometry_msgs::msg::Pose & object_pose,
    const std::optional<double> & distance_from_last_update)
  {
    if (!publish_updates_) {
      return;
    }

    std_msgs::msg::String message;
    message.data = build_status_message(
      event, frame_id, object_pose, distance_from_last_update);
    status_publisher_->publish(message);
  }

  std::string build_status_message(
    const std::string & event,
    const std::string & frame_id,
    const geometry_msgs::msg::Pose & object_pose,
    const std::optional<double> & distance_from_last_update) const
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6);
    stream << "event=" << event;
    stream << ";object_id=" << object_id_;
    stream << ";frame=" << frame_id;
    stream << ";x=" << object_pose.position.x;
    stream << ";y=" << object_pose.position.y;
    stream << ";z=" << object_pose.position.z;
    stream << ";distance_from_last_update=";
    if (distance_from_last_update.has_value()) {
      stream << distance_from_last_update.value();
    } else {
      stream << "none";
    }
    stream << ";min_update_distance=" << min_update_distance_;
    stream << ";ready=" << (ready_ ? "true" : "false");
    return stream.str();
  }

  rclcpp::Node::SharedPtr node_;
  std::string input_pose_topic_;
  std::string object_id_;
  double size_x_;
  double size_y_;
  double size_z_;
  double x_offset_;
  double y_offset_;
  double z_offset_;
  double min_update_distance_;
  bool publish_updates_;
  bool ready_{false};
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  std::optional<geometry_msgs::msg::Pose> last_applied_pose_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "dynamic_target_scene_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto dynamic_target_scene = std::make_shared<DynamicTargetSceneNode>(node);

  rclcpp::spin(node);
  dynamic_target_scene.reset();
  rclcpp::shutdown();
  return 0;
}
