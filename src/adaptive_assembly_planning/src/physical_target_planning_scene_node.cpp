#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "adaptive_assembly_planning/linear_path_validation.hpp"
#include "adaptive_assembly_planning/target_scene_contract.hpp"

namespace aap = adaptive_assembly_planning;

class PhysicalTargetPlanningSceneNode
{
public:
  explicit PhysicalTargetPlanningSceneNode(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    object_id_(param<std::string>("target_object_id", "target_object")),
    target_pose_topic_(param<std::string>("target_pose_topic", "/target_pose")),
    planning_frame_(param<std::string>("planning_frame", "panda_link0")),
    radius_(param<double>("target_radius", 0.035)),
    height_(param<double>("target_height", 0.10)),
    stale_timeout_sec_(param<double>("target_pose_stale_timeout_sec", 1.0)),
    ready_topic_(param<std::string>("ready_topic", "/physical_target_planning_scene_ready")),
    status_topic_(param<std::string>("status_topic", "/physical_target_planning_scene_status")),
    lock_status_topic_(param<std::string>("plan_lock_status_topic",
    "/assembly_sequence_plan_lock_status")),
    allowed_links_(split_csv(param<std::string>(
        "target_allowed_collision_links_csv", "panda_leftfinger,panda_rightfinger"))),
    tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_)
  {
    if (object_id_.empty() || planning_frame_.empty() || !(radius_ > 0.0) || !(height_ > 0.0) ||
      !std::isfinite(radius_) || !std::isfinite(height_) || stale_timeout_sec_ <= 0.0)
    {throw std::invalid_argument("invalid_target_scene_parameters");}
    if (allowed_links_ != std::vector<std::string>({"panda_leftfinger", "panda_rightfinger"})) {
      throw std::invalid_argument("invalid_target_acm_allowlist");
    }
    robot_model_loader::RobotModelLoader loader(node_, "robot_description");
    robot_model_ = loader.getModel();
    if (!robot_model_) {throw std::invalid_argument("target_scene_robot_model_unavailable");}
    for (const auto & link : allowed_links_) {
      if (!robot_model_->hasLinkModel(link)) {
        throw std::invalid_argument("target_scene_finger_link_missing:" + link);
      }
    }
    ready_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
      ready_topic_, rclcpp::QoS(1).transient_local().reliable());
    status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(10).transient_local().reliable());
    client_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    get_scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene", rclcpp::ServicesQoS(), client_callback_group_);
    apply_scene_client_ = node_->create_client<moveit_msgs::srv::ApplyPlanningScene>(
      "/apply_planning_scene", rclcpp::ServicesQoS(), client_callback_group_);
    pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      target_pose_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr pose) {pose_callback(*pose);});
    lock_subscription_ = node_->create_subscription<std_msgs::msg::String>(
      lock_status_topic_, rclcpp::QoS(10).reliable().durability_volatile(),
      [this](const std_msgs::msg::String::SharedPtr message) {lock_callback(message->data);});
    publish_ready(false);
    publish_status("waiting", "waiting_for_target_pose", "", 0.0);
  }

private:
  template<typename T>
  T param(const std::string & name, const T & value)
  {
    if (!node_->has_parameter(name)) {node_->declare_parameter<T>(name, value);}
    return node_->get_parameter(name).get_value<T>();
  }

  static std::vector<std::string> split_csv(const std::string & text)
  {
    std::vector<std::string> result;
    std::istringstream stream(text);
    std::string item;
    while (std::getline(stream, item, ',')) {
      item.erase(0, item.find_first_not_of(" \t"));
      if (!item.empty()) {item.erase(item.find_last_not_of(" \t") + 1); result.push_back(item);}
    }
    return result;
  }

  static std::string field(const std::string & status, const std::string & key)
  {
    std::istringstream stream(status);
    std::string fragment;
    while (std::getline(stream, fragment, ';')) {
      const auto equal = fragment.find('=');
      if (equal != std::string::npos && fragment.substr(0, equal) == key) {
        return fragment.substr(equal + 1);
      }
    }
    return "";
  }

  void lock_callback(const std::string & status)
  {
    if (locked_ || field(status, "mode") != "sequence_plan_lock") {return;}
    const std::string event = field(status, "event");
    if (event == "planning_started") {
      planning_frozen_ = true;
      publish_status("planning_snapshot_frozen", "", source_frame_, last_pose_age_sec_);
      return;
    }
    if (event == "failure") {
      planning_frozen_ = false;
      publish_status("planning_snapshot_released", field(status, "reason"), source_frame_,
        last_pose_age_sec_);
      return;
    }
    if (event != "locked" || field(status, "locked") != "true") {return;}
    if (!ready_) {
      publish_status("failure", "lock_before_target_ready", source_frame_, last_pose_age_sec_);
      return;
    }
    try {plan_id_ = std::stoull(field(status, "plan_id"));} catch (...) {
      publish_status("failure", "invalid_plan_lock_status", source_frame_, last_pose_age_sec_);
      return;
    }
    planning_frozen_ = true;
    locked_ = true;
    publish_status("locked", "", source_frame_, last_pose_age_sec_);
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped & input)
  {
    if (locked_ || planning_frozen_) {return;}
    if (input.header.frame_id.empty()) {
      reject("empty_source_frame", input.header.frame_id, 0.0); return;
    }
    if (!aap::finite_pose(input.pose) ||
      !(aap::quaternion_norm(input.pose.orientation) > 1.0e-12) ||
      std::abs(aap::quaternion_norm(input.pose.orientation) - 1.0) > 1.0e-3)
    {reject("invalid_target_pose", input.header.frame_id, 0.0); return;}
    double age = 0.0;
    const rclcpp::Time stamp(input.header.stamp);
    if (stamp.nanoseconds() > 0) {age = (node_->now() - stamp).seconds();}
    if (age < -0.05 || age > stale_timeout_sec_) {
      reject("stale_target_pose", input.header.frame_id, age); return;
    }
    geometry_msgs::msg::PoseStamped transformed;
    try {
      transformed = tf_buffer_.transform(
        input, planning_frame_, tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN(node_->get_logger(), "Target pose TF failed: %s", error.what());
      reject("target_pose_tf_failed", input.header.frame_id, age); return;
    }
    apply_target(transformed, input.header.frame_id, age);
  }

  void reject(const std::string & reason, const std::string & source_frame, double age)
  {
    publish_ready(false);
    publish_status("failure", reason, source_frame, age);
  }

  moveit_msgs::msg::CollisionObject make_object(const geometry_msgs::msg::PoseStamped & pose) const
  {
    return aap::make_target_cylinder(object_id_, planning_frame_, radius_, height_, pose);
  }

  bool wait_service(const rclcpp::ClientBase::SharedPtr & client)
  {
    return client->wait_for_service(std::chrono::seconds(2));
  }

  void apply_target(
    const geometry_msgs::msg::PoseStamped & pose, const std::string & source_frame, double age)
  {
    if (!wait_service(get_scene_client_) || !wait_service(apply_scene_client_)) {
      reject("planning_scene_service_unavailable", source_frame, age); return;
    }
    auto get_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    get_request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto get_future = get_scene_client_->async_send_request(get_request);
    if (get_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      reject("planning_scene_query_timeout", source_frame, age); return;
    }
    auto scene = get_future.get()->scene;
    auto & acm = scene.allowed_collision_matrix;
    aap::configure_target_acm(acm, object_id_, allowed_links_);
    aap::set_acm_pair(acm, "panda_link0", "work_table", true);
    scene.is_diff = true;
    scene.robot_state.is_diff = true;
    scene.world.collision_objects.clear();
    scene.world.collision_objects.push_back(make_object(pose));
    auto apply_request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    apply_request->scene = scene;
    auto apply_future = apply_scene_client_->async_send_request(apply_request);
    if (apply_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready ||
      !apply_future.get()->success)
    {reject("planning_scene_apply_failed", source_frame, age); return;}
    ready_ = true; source_frame_ = source_frame; last_pose_age_sec_ = age; locked_pose_ = pose;
    publish_ready(true);
    publish_status("applied", "", source_frame, age);
  }

  void publish_ready(bool ready)
  {
    ready_ = ready;
    std_msgs::msg::Bool message; message.data = ready; ready_publisher_->publish(message);
  }

  void publish_status(
    const std::string & event, const std::string & reason,
    const std::string & source_frame, double age)
  {
    std_msgs::msg::String message;
    std::ostringstream out;
    out << "event=" << event << ";mode=physical_target_planning_scene"
        << ";object_id=" << object_id_ << ";source_frame=" << source_frame
        << ";planning_frame=" << planning_frame_ << ";pose_age_sec=" << age
        << ";radius=" << radius_ << ";height=" << height_
        << ";allowed_collision_links=panda_leftfinger,panda_rightfinger"
        << ";locked=" << (locked_ ? "true" : "false");
    if (plan_id_ > 0) {out << ";plan_id=" << plan_id_;}
    if (ready_) {
      out << ";target_x=" << locked_pose_.pose.position.x
          << ";target_y=" << locked_pose_.pose.position.y
          << ";target_z=" << locked_pose_.pose.position.z;
    }
    if (!reason.empty()) {out << ";reason=" << reason;}
    message.data = out.str(); status_publisher_->publish(message);
  }

  rclcpp::Node::SharedPtr node_;
  std::string object_id_, target_pose_topic_, planning_frame_;
  double radius_, height_, stale_timeout_sec_;
  std::string ready_topic_, status_topic_, lock_status_topic_;
  std::vector<std::string> allowed_links_;
  bool ready_{false}, locked_{false}, planning_frozen_{false};
  std::uint64_t plan_id_{0};
  std::string source_frame_;
  double last_pose_age_sec_{0.0};
  geometry_msgs::msg::PoseStamped locked_pose_;
  moveit::core::RobotModelConstPtr robot_model_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lock_subscription_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_client_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "physical_target_planning_scene_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  try {
    auto target_scene = std::make_shared<PhysicalTargetPlanningSceneNode>(node);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node); executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "%s", error.what());
    rclcpp::shutdown(); return 2;
  }
  rclcpp::shutdown(); return 0;
}
