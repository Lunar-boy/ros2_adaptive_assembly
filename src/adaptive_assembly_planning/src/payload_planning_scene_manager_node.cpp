#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
Eigen::Isometry3d pose_to_eigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y,
    pose.orientation.z);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  if (q.norm() > 1.0e-12) {result.linear() = q.normalized().toRotationMatrix();}
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return result;
}

geometry_msgs::msg::Pose eigen_to_pose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Quaterniond q(transform.rotation());
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

bool close_pose(const geometry_msgs::msg::Pose & lhs, const geometry_msgs::msg::Pose & rhs)
{
  const Eigen::Isometry3d delta = pose_to_eigen(lhs).inverse() * pose_to_eigen(rhs);
  return delta.translation().norm() <= 1.0e-5 &&
         Eigen::AngleAxisd(delta.rotation()).angle() <= 1.0e-5;
}
}  // namespace

class PayloadPlanningSceneManager
{
public:
  explicit PayloadPlanningSceneManager(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    object_id_(param<std::string>("object_id", "target_object")),
    attachment_link_(param<std::string>("attachment_link", "assembly_tcp")),
    planning_frame_(param<std::string>("planning_frame", "panda_link0")),
    gazebo_pose_topic_(param<std::string>("gazebo_pose_topic", "/gazebo_target_object_pose")),
    freshness_timeout_(param<double>("pose_freshness_timeout_sec", 0.5)),
    command_topic_(param<std::string>("command_topic", "/payload_attachment_command")),
    status_topic_(param<std::string>("status_topic", "/payload_attachment_status")),
    ready_topic_(param<std::string>("ready_topic", "/payload_attachment_ready")),
    grasp_verified_topic_(param<std::string>("grasp_verified_topic", "/grasp_verified")),
    touch_links_(split_csv(param<std::string>("touch_links",
      "panda_leftfinger,panda_rightfinger"))),
    tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_)
  {
    if (object_id_.empty() || attachment_link_.empty() || planning_frame_.empty() ||
      freshness_timeout_ <= 0.0 ||
      touch_links_ != std::vector<std::string>({"panda_leftfinger", "panda_rightfinger"}))
    {
      throw std::invalid_argument("invalid_payload_attachment_configuration");
    }
    robot_model_loader::RobotModelLoader loader(node_, "robot_description");
    robot_model_ = loader.getModel();
    if (!robot_model_ || !robot_model_->hasLinkModel(attachment_link_)) {
      throw std::invalid_argument("attachment_link_not_in_robot_model");
    }
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    get_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene", rmw_qos_profile_services_default, callback_group_);
    apply_client_ = node_->create_client<moveit_msgs::srv::ApplyPlanningScene>(
      "/apply_planning_scene", rmw_qos_profile_services_default, callback_group_);
    const auto retained = rclcpp::QoS(1).transient_local().reliable();
    status_publisher_ = node_->create_publisher<std_msgs::msg::String>(status_topic_, retained);
    ready_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(ready_topic_, retained);
    command_subscription_ = node_->create_subscription<std_msgs::msg::String>(
      command_topic_, 10, [this](const std_msgs::msg::String::SharedPtr msg) {command(*msg);});
    grasp_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
      grasp_verified_topic_, retained,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        grasp_verified_ = msg->data;
        if (grasp_verified_ && attach_pending_ && !busy_) {
          attach_pending_ = false; busy_ = true; attach(); busy_ = false;
        }
      });
    pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      gazebo_pose_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        latest_gazebo_pose_ = *msg; latest_pose_receive_time_ = node_->now();
      });
    joint_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 20,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->name.size() == msg->position.size() &&
        std::all_of(msg->position.begin(), msg->position.end(), [](double value) {
          return std::isfinite(value);
          }))
        {
          latest_joint_state_ = *msg; latest_joint_receive_time_ = node_->now();
        }
      });
    publish_ready(false);
    publish_status("waiting", "none", "waiting_for_command", "");
  }

private:
  template<typename T> T param(const std::string & name, const T & value)
  {
    if (!node_->has_parameter(name)) {node_->declare_parameter<T>(name, value);}
    return node_->get_parameter(name).get_value<T>();
  }

  static std::vector<std::string> split_csv(const std::string & text)
  {
    std::vector<std::string> values;
    std::stringstream stream(text); std::string value;
    while (std::getline(stream, value, ',')) {
      value.erase(0, value.find_first_not_of(" \t"));
      if (!value.empty()) {value.erase(value.find_last_not_of(" \t") + 1); values.push_back(value);}
    }
    return values;
  }

  static std::string field(const std::string & text, const std::string & key)
  {
    std::stringstream stream(text); std::string part;
    while (std::getline(stream, part, ';')) {
      const auto equal = part.find('=');
      if (equal != std::string::npos && part.substr(0, equal) == key) {
        return part.substr(equal + 1);
      }
    }
    return "";
  }

  void command(const std_msgs::msg::String & message)
  {
    const auto requested = field(message.data, "command");
    command_id_ = field(message.data, "command_id");
    if (busy_) {fail(requested, "operation_in_progress"); return;}
    busy_ = true;
    if (requested == "attach") {attach();} else if (requested == "detach") {detach();} else {
      fail(requested, "invalid_command");
    }
    busy_ = false;
  }

  std::optional<moveit_msgs::msg::PlanningScene> get_scene()
  {
    if (!get_client_->wait_for_service(std::chrono::seconds(2))) {return std::nullopt;}
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
      moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
      moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto future = get_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      return std::nullopt;
    }
    return future.get()->scene;
  }

  bool apply(moveit_msgs::msg::PlanningScene scene)
  {
    if (!apply_client_->wait_for_service(std::chrono::seconds(2))) {return false;}
    scene.is_diff = true; scene.robot_state.is_diff = true;
    auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    request->scene = std::move(scene);
    auto future = apply_client_->async_send_request(request);
    return future.wait_for(std::chrono::seconds(2)) == std::future_status::ready &&
           future.get()->success;
  }

  static std::vector<moveit_msgs::msg::CollisionObject> world_matches(
    const moveit_msgs::msg::PlanningScene & scene, const std::string & id)
  {
    std::vector<moveit_msgs::msg::CollisionObject> result;
    for (const auto & object : scene.world.collision_objects) {
      if (object.id == id && object.operation != moveit_msgs::msg::CollisionObject::REMOVE) {
        result.push_back(object);
      }
    }
    return result;
  }

  static std::vector<moveit_msgs::msg::AttachedCollisionObject> attached_matches(
    const moveit_msgs::msg::PlanningScene & scene, const std::string & id)
  {
    std::vector<moveit_msgs::msg::AttachedCollisionObject> result;
    for (const auto & object : scene.robot_state.attached_collision_objects) {
      if (object.object.id == id &&
        object.object.operation != moveit_msgs::msg::CollisionObject::REMOVE)
      {
        result.push_back(object);
      }
    }
    return result;
  }

  bool exact_cylinder(const moveit_msgs::msg::CollisionObject & object) const
  {
    return object.primitives.size() == 1 && object.primitive_poses.size() == 1 &&
           object.primitives[0].type == shape_msgs::msg::SolidPrimitive::CYLINDER &&
           object.primitives[0].dimensions.size() == 2 &&
           std::abs(
      object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] - 0.035) <
           1e-9 &&
           std::abs(
      object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] - 0.10) <
           1e-9;
  }

  bool exact_finger_only_acm(const moveit_msgs::msg::PlanningScene & scene) const
  {
    const auto & names = scene.allowed_collision_matrix.entry_names;
    const auto found = std::find(names.begin(), names.end(), object_id_);
    if (found == names.end()) {return false;}
    const auto index = static_cast<std::size_t>(std::distance(names.begin(), found));
    if (index >= scene.allowed_collision_matrix.entry_values.size()) {return false;}
    std::vector<std::string> allowed;
    const auto & enabled = scene.allowed_collision_matrix.entry_values[index].enabled;
    for (std::size_t i = 0; i < std::min(names.size(), enabled.size()); ++i) {
      if (enabled[i] && names[i] != object_id_) {allowed.push_back(names[i]);}
    }
    std::sort(allowed.begin(), allowed.end());
    auto expected = touch_links_; std::sort(expected.begin(), expected.end());
    return allowed == expected;
  }

  void attach()
  {
    publish_ready(false);
    if (!grasp_verified_) {
      attach_pending_ = true;
      publish_status("waiting", "attach", "grasp_not_verified", "world");
      return;
    }
    auto scene = get_scene();
    if (!scene) {fail("attach", "planning_scene_query_failed"); return;}
    const auto world = world_matches(*scene, object_id_);
    const auto attached = attached_matches(*scene, object_id_);
    if (world.size() != 1 || !attached.empty()) {
      fail("attach", world.empty() ? "world_object_missing" : "object_count_invalid"); return;
    }
    if (!exact_cylinder(world.front()) || world.front().header.frame_id.empty()) {
      fail("attach", "world_object_geometry_invalid"); return;
    }
    moveit::core::RobotState state(robot_model_);
    moveit::core::robotStateMsgToRobotState(scene->robot_state, state, true);
    if (!latest_joint_state_ ||
      (node_->now() - latest_joint_receive_time_).seconds() > freshness_timeout_)
    {
      fail("attach", "current_robot_state_stale"); return;
    }
    state.setVariablePositions(latest_joint_state_->name, latest_joint_state_->position);
    state.updateLinkTransforms();
    Eigen::Isometry3d global_from_object_frame = Eigen::Isometry3d::Identity();
    if (world.front().header.frame_id != robot_model_->getModelFrame()) {
      if (!robot_model_->hasLinkModel(world.front().header.frame_id)) {
        fail("attach", "world_object_frame_invalid"); return;
      }
      global_from_object_frame = state.getGlobalLinkTransform(
        world.front().header.frame_id);
    }
    const Eigen::Isometry3d link_inverse =
      state.getGlobalLinkTransform(attachment_link_).inverse();
    moveit_msgs::msg::AttachedCollisionObject payload;
    payload.link_name = attachment_link_;
    payload.touch_links = touch_links_;
    payload.object = world.front();
    payload.object.header.frame_id = attachment_link_;
    const Eigen::Isometry3d object_pose = pose_to_eigen(payload.object.pose);
    for (auto & pose : payload.object.primitive_poses) {
      pose = eigen_to_pose(
        link_inverse * global_from_object_frame * object_pose * pose_to_eigen(pose));
    }
    payload.object.pose = geometry_msgs::msg::Pose();
    payload.object.pose.orientation.w = 1.0;
    payload.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    measured_relative_pose_ = payload.object.primitive_poses.front();
    moveit_msgs::msg::PlanningScene diff;
    // PlanningScene's AttachedCollisionObject ADD atomically consumes the
    // same-ID world object; an explicit world REMOVE would remove it twice.
    diff.robot_state.attached_collision_objects.push_back(payload);
    if (!apply(diff)) {fail("attach", "planning_scene_apply_failed"); return;}
    auto verified = get_scene();
    if (!verified) {fail("attach", "planning_scene_verify_query_failed"); return;}
    const auto verified_world = world_matches(*verified, object_id_);
    const auto verified_attached = attached_matches(*verified, object_id_);
    if (!verified_world.empty() || verified_attached.size() != 1 ||
      verified_attached.front().link_name != attachment_link_ ||
      verified_attached.front().touch_links != touch_links_ ||
      !exact_cylinder(verified_attached.front().object) ||
      !exact_finger_only_acm(*verified) ||
      !close_pose(
        eigen_to_pose(
          pose_to_eigen(verified_attached.front().object.pose) *
          pose_to_eigen(verified_attached.front().object.primitive_poses.front())),
        measured_relative_pose_))
    {fail("attach", "attachment_verification_failed"); return;}
    attached_geometry_ = verified_attached.front().object;
    attached_ = true; publish_ready(true); publish_status("success", "attach", "", "attached");
  }

  void detach()
  {
    publish_ready(false);
    auto scene = get_scene();
    if (!scene) {fail("detach", "planning_scene_query_failed"); return;}
    const auto world = world_matches(*scene, object_id_);
    const auto attached = attached_matches(*scene, object_id_);
    if (!world.empty() || attached.size() != 1) {fail("detach", "object_count_invalid"); return;}
    if (!exact_cylinder(attached.front().object)) {
      fail("detach", "attached_geometry_invalid"); return;
    }
    if (!latest_gazebo_pose_) {fail("detach", "gazebo_pose_missing"); return;}
    const double age = (node_->now() - latest_pose_receive_time_).seconds();
    if (age < 0.0 || age > freshness_timeout_) {fail("detach", "gazebo_pose_stale"); return;}
    geometry_msgs::msg::PoseStamped observed;
    try {
      observed = tf_buffer_.transform(*latest_gazebo_pose_, planning_frame_,
        tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException &) {
      fail("detach", "gazebo_pose_transform_failed"); return;
    }
    moveit_msgs::msg::AttachedCollisionObject remove_attached;
    remove_attached.link_name = attachment_link_; remove_attached.object.id = object_id_;
    remove_attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    auto restored = attached.front().object;
    restored.header.frame_id = planning_frame_;
    restored.pose = geometry_msgs::msg::Pose();
    restored.pose.orientation.w = 1.0;
    restored.primitive_poses.assign(1, observed.pose);
    restored.operation = moveit_msgs::msg::CollisionObject::ADD;
    moveit_msgs::msg::PlanningScene diff;
    diff.robot_state.attached_collision_objects.push_back(remove_attached);
    diff.world.collision_objects.push_back(restored);
    // The target ACM row is intentionally retained from the original world object;
    // attachment touch links never broaden it.
    if (!apply(diff)) {fail("detach", "planning_scene_apply_failed"); return;}
    auto verified = get_scene();
    if (!verified) {fail("detach", "planning_scene_verify_query_failed"); return;}
    const auto verified_world = world_matches(*verified, object_id_);
    const auto verified_attached = attached_matches(*verified, object_id_);
    if (verified_world.size() != 1 || !verified_attached.empty() ||
      !exact_cylinder(verified_world.front()) ||
      !exact_finger_only_acm(*verified) ||
      !close_pose(
        eigen_to_pose(
          pose_to_eigen(verified_world.front().pose) *
          pose_to_eigen(verified_world.front().primitive_poses.front())),
        observed.pose))
    {fail("detach", "detachment_verification_failed"); return;}
    attached_ = false; publish_status("success", "detach", "", "world");
  }

  void fail(const std::string & operation, const std::string & reason)
  {
    publish_ready(false);
    publish_status("failure", operation, reason, attached_ ? "attached" : "world");
  }

  void publish_ready(bool ready)
  {
    std_msgs::msg::Bool message; message.data = ready; ready_publisher_->publish(message);
  }

  void publish_status(
    const std::string & event, const std::string & operation,
    const std::string & reason, const std::string & state)
  {
    std_msgs::msg::String message; std::ostringstream out;
    out << "event=" << event << ";mode=payload_attachment;operation=" << operation
        << ";command_id=" << command_id_ << ";object_id=" << object_id_
        << ";payload_state=" << state << ";attachment_link=" << attachment_link_
        << ";touch_links=panda_leftfinger,panda_rightfinger"
        << ";geometry=cylinder;radius=0.035;height=0.10"
        << ";relative_x=" << measured_relative_pose_.position.x
        << ";relative_y=" << measured_relative_pose_.position.y
        << ";relative_z=" << measured_relative_pose_.position.z;
    if (!reason.empty()) {out << ";reason=" << reason;}
    message.data = out.str(); status_publisher_->publish(message);
    RCLCPP_INFO(node_->get_logger(), "Payload status: %s", message.data.c_str());
  }

  rclcpp::Node::SharedPtr node_;
  std::string object_id_, attachment_link_, planning_frame_, gazebo_pose_topic_;
  double freshness_timeout_;
  std::string command_topic_, status_topic_, ready_topic_, grasp_verified_topic_, command_id_;
  std::vector<std::string> touch_links_;
  bool grasp_verified_{false}, attached_{false}, busy_{false}, attach_pending_{false};
  std::optional<geometry_msgs::msg::PoseStamped> latest_gazebo_pose_;
  std::optional<sensor_msgs::msg::JointState> latest_joint_state_;
  rclcpp::Time latest_pose_receive_time_;
  rclcpp::Time latest_joint_receive_time_;
  geometry_msgs::msg::Pose measured_relative_pose_;
  moveit_msgs::msg::CollisionObject attached_geometry_;
  moveit::core::RobotModelConstPtr robot_model_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_client_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasp_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("payload_planning_scene_manager_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  try {
    auto manager = std::make_shared<PayloadPlanningSceneManager>(node);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node); executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "%s", error.what()); rclcpp::shutdown(); return 2;
  }
  rclcpp::shutdown(); return 0;
}
