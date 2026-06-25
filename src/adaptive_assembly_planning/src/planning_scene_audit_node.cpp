#include <algorithm>
#include <chrono>
#include <cctype>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class PlanningSceneAuditNode
{
public:
  explicit PlanningSceneAuditNode(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    expected_object_ids_text_(declare_parameter<std::string>(
        "expected_object_ids", "work_table,target_support,target_object_dynamic")),
    audit_period_sec_(declare_parameter<double>("audit_period_sec", 2.0)),
    status_topic_(declare_parameter<std::string>(
        "status_topic", "/planning_scene_audit_status")),
    ready_topic_(declare_parameter<std::string>(
        "ready_topic", "/planning_scene_audit_ready")),
    expected_object_ids_(split_csv(expected_object_ids_text_))
  {
    if (audit_period_sec_ <= 0.0) {
      RCLCPP_WARN(
        node_->get_logger(),
        "audit_period_sec=%.3f is invalid; using 2.0 seconds.",
        audit_period_sec_);
      audit_period_sec_ = 2.0;
    }

    ready_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
      ready_topic_, 10);
    status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
      status_topic_, 10);

    audit_timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(audit_period_sec_)),
      [this]() {
        run_audit();
      });

    RCLCPP_INFO(
      node_->get_logger(),
      "PlanningScene audit node ready: expected='%s', audit_period_sec=%.3f, "
      "status_topic='%s', ready_topic='%s'. Introspection only; this node does "
      "not modify the PlanningScene or execute trajectories.",
      expected_object_ids_text_.c_str(), audit_period_sec_, status_topic_.c_str(),
      ready_topic_.c_str());
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

  static std::vector<std::string> split_csv(const std::string & input)
  {
    std::vector<std::string> values;
    std::stringstream stream(input);
    std::string item;
    while (std::getline(stream, item, ',')) {
      item.erase(
        item.begin(),
        std::find_if(item.begin(), item.end(), [](const unsigned char character) {
          return !std::isspace(character);
        }));
      item.erase(
        std::find_if(item.rbegin(), item.rend(), [](const unsigned char character) {
          return !std::isspace(character);
        }).base(),
        item.end());
      if (!item.empty()) {
        values.push_back(item);
      }
    }
    return values;
  }

  static std::string join(const std::vector<std::string> & values)
  {
    if (values.empty()) {
      return "none";
    }

    std::ostringstream stream;
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (index > 0) {
        stream << ",";
      }
      stream << values[index];
    }
    return stream.str();
  }

  void run_audit()
  {
    const auto objects = planning_scene_interface_.getObjects(expected_object_ids_);

    std::vector<std::string> present;
    std::vector<std::string> missing;
    for (const auto & object_id : expected_object_ids_) {
      if (objects.find(object_id) != objects.end()) {
        present.push_back(object_id);
      } else {
        missing.push_back(object_id);
      }
    }

    const bool all_present = !expected_object_ids_.empty() && missing.empty();
    publish_ready(all_present);
    publish_status(present, missing, all_present);

    RCLCPP_INFO(
      node_->get_logger(),
      "PlanningScene audit: expected='%s', present='%s', missing='%s', "
      "all_present=%s.",
      join(expected_object_ids_).c_str(), join(present).c_str(),
      join(missing).c_str(), all_present ? "true" : "false");
  }

  void publish_ready(const bool all_present)
  {
    std_msgs::msg::Bool message;
    message.data = all_present;
    ready_publisher_->publish(message);
  }

  void publish_status(
    const std::vector<std::string> & present,
    const std::vector<std::string> & missing,
    const bool all_present)
  {
    std_msgs::msg::String message;
    std::ostringstream stream;
    stream << "event=audit";
    stream << ";expected=" << join(expected_object_ids_);
    stream << ";present=" << join(present);
    stream << ";missing=" << join(missing);
    stream << ";all_present=" << (all_present ? "true" : "false");
    message.data = stream.str();
    status_publisher_->publish(message);
  }

  rclcpp::Node::SharedPtr node_;
  std::string expected_object_ids_text_;
  double audit_period_sec_;
  std::string status_topic_;
  std::string ready_topic_;
  std::vector<std::string> expected_object_ids_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr audit_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "planning_scene_audit_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto audit_node = std::make_shared<PlanningSceneAuditNode>(node);

  rclcpp::spin(node);
  audit_node.reset();
  rclcpp::shutdown();
  return 0;
}
