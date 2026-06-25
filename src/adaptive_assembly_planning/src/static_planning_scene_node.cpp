#include <algorithm>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

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
    status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
      "/static_planning_scene_status",
      rclcpp::QoS(10).transient_local().reliable());

    clear_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/clear_static_planning_scene",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>/* request */,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        clear_service_callback(response);
      });

    reapply_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/reapply_static_planning_scene",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>/* request */,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        reapply_service_callback(response);
      });

    const auto delay = std::chrono::duration<double>(
      std::max(0.0, apply_delay_sec_));
    apply_timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(delay),
      [this]() {
        apply_static_objects(true);
      });

    RCLCPP_INFO(
      node_->get_logger(),
      "Static PlanningScene node ready for frame '%s'. Static collision "
      "objects only; trajectory execution is disabled. Services available: "
      "/clear_static_planning_scene and /reapply_static_planning_scene.",
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

  void apply_static_objects(const bool cancel_timer)
  {
    if (cancel_timer && apply_timer_) {
      apply_timer_->cancel();
    }

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
      publish_status("no_objects_enabled");
      return;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Applying %zu static PlanningScene collision object(s) in frame '%s'.",
      collision_objects.size(), planning_frame_.c_str());

    const bool applied = planning_scene_interface_.applyCollisionObjects(
      collision_objects);
    publish_ready(applied);
    publish_status(applied ? "applied" : "apply_failed");

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

  void clear_service_callback(
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const auto object_ids = enabled_object_ids();
    if (object_ids.empty()) {
      ready_ = false;
      publish_ready(false);
      publish_status("cleared");
      response->success = true;
      response->message = "No static PlanningScene collision objects are enabled.";
      RCLCPP_WARN(
        node_->get_logger(),
        "Static PlanningScene clear requested, but no static objects are enabled.");
      return;
    }

    try {
      planning_scene_interface_.removeCollisionObjects(object_ids);
      ready_ = false;
      publish_ready(false);
      publish_status("cleared");
      response->success = true;
      response->message = "Removed static PlanningScene objects: " +
        join_object_ids(object_ids);

      RCLCPP_INFO(
        node_->get_logger(),
        "Cleared static PlanningScene objects: %s.",
        join_object_ids(object_ids).c_str());
    } catch (const std::exception & exception) {
      publish_status("clear_failed");
      response->success = false;
      response->message = "Failed to clear static PlanningScene objects: " +
        std::string(exception.what());
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to clear static PlanningScene objects: %s",
        exception.what());
    }
  }

  void reapply_service_callback(
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const auto object_ids = enabled_object_ids();
    if (object_ids.empty()) {
      ready_ = false;
      publish_ready(false);
      publish_status("no_objects_enabled");
      response->success = true;
      response->message = "No static PlanningScene collision objects are enabled.";
      RCLCPP_WARN(
        node_->get_logger(),
        "Static PlanningScene reapply requested, but no static objects are enabled.");
      return;
    }

    try {
      apply_static_objects(false);
      response->success = ready_;
      response->message = ready_ ?
        "Reapplied static PlanningScene objects: " + join_object_ids(object_ids) :
        "Failed to reapply static PlanningScene objects: " + join_object_ids(object_ids);
    } catch (const std::exception & exception) {
      ready_ = false;
      publish_ready(false);
      publish_status("apply_failed");
      response->success = false;
      response->message = "Failed to reapply static PlanningScene objects: " +
        std::string(exception.what());
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to reapply static PlanningScene objects: %s",
        exception.what());
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
    ready_ = ready;
    std_msgs::msg::Bool message;
    message.data = ready;
    ready_publisher_->publish(message);
  }

  void publish_status(const std::string & event)
  {
    std_msgs::msg::String message;
    message.data =
      "event=" + event +
      ";object_ids=" + join_object_ids(enabled_object_ids()) +
      ";frame=" + planning_frame_ +
      ";ready=" + std::string(ready_ ? "true" : "false");
    status_publisher_->publish(message);

    RCLCPP_INFO(
      node_->get_logger(),
      "Static PlanningScene status: %s",
      message.data.c_str());
  }

  std::vector<std::string> enabled_object_ids() const
  {
    std::vector<std::string> object_ids;
    if (add_work_table_) {
      object_ids.push_back("work_table");
    }
    if (add_target_support_) {
      object_ids.push_back("target_support");
    }
    return object_ids;
  }

  std::string join_object_ids(const std::vector<std::string> & object_ids) const
  {
    if (object_ids.empty()) {
      return "none";
    }

    std::ostringstream stream;
    for (std::size_t index = 0; index < object_ids.size(); ++index) {
      if (index > 0) {
        stream << ",";
      }
      stream << object_ids[index];
    }
    return stream.str();
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
  bool ready_{false};
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reapply_service_;
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
