#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

class PandaAssemblyPoseAdapter
{
public:
  explicit PandaAssemblyPoseAdapter(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    input_topic_(declare_parameter<std::string>("input_topic", "/assembly_pose")),
    output_topic_(declare_parameter<std::string>("output_topic", "/panda_assembly_pose")),
    output_frame_id_(declare_parameter<std::string>("output_frame_id", "")),
    use_tf_transform_(declare_parameter<bool>("use_tf_transform", false)),
    target_frame_id_(declare_parameter<std::string>("target_frame_id", "panda_link0")),
    tf_lookup_timeout_sec_(declare_parameter<double>("tf_lookup_timeout_sec", 0.2)),
    status_topic_(declare_parameter<std::string>(
        "status_topic", "/panda_assembly_pose_adapter_status")),
    x_offset_(declare_parameter<double>("x_offset", 0.0)),
    y_offset_(declare_parameter<double>("y_offset", 0.0)),
    z_offset_(declare_parameter<double>("z_offset", 0.0)),
    use_fixed_orientation_(declare_parameter<bool>("use_fixed_orientation", true)),
    fixed_qx_(declare_parameter<double>("fixed_qx", 1.0)),
    fixed_qy_(declare_parameter<double>("fixed_qy", 0.0)),
    fixed_qz_(declare_parameter<double>("fixed_qz", 0.0)),
    fixed_qw_(declare_parameter<double>("fixed_qw", 0.0)),
    normalize_quaternion_(declare_parameter<bool>("normalize_quaternion", true))
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    adapted_pose_publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_, 10);
    status_publisher_ = node_->create_publisher<std_msgs::msg::String>(status_topic_, 10);
    assembly_subscription_ =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        assembly_pose_callback(*message);
      });

    RCLCPP_INFO(
      node_->get_logger(),
      "Panda assembly pose adapter ready: input_topic='%s', output_topic='%s', "
      "output_frame_id='%s', use_tf_transform=%s, target_frame_id='%s', "
      "tf_lookup_timeout_sec=%.3f, status_topic='%s', use_fixed_orientation=%s. "
      "Pose adaptation only; no trajectory execution.",
      input_topic_.c_str(), output_topic_.c_str(), output_frame_id_.c_str(),
      use_tf_transform_ ? "true" : "false", target_frame_id_.c_str(),
      tf_lookup_timeout_sec_, status_topic_.c_str(),
      use_fixed_orientation_ ? "true" : "false");
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

  void assembly_pose_callback(const geometry_msgs::msg::PoseStamped & input_pose)
  {
    const auto & input_position = input_pose.pose.position;
    RCLCPP_INFO(
      node_->get_logger(),
      "Received task assembly pose: frame='%s', x=%.3f, y=%.3f, z=%.3f",
      input_pose.header.frame_id.c_str(), input_position.x, input_position.y,
      input_position.z);

    if (input_pose.header.frame_id.empty()) {
      publish_status(
        "skipped_empty_frame", input_pose.header.frame_id, "", input_position.x,
        input_position.y, input_position.z);
      RCLCPP_WARN(
        node_->get_logger(),
        "Skipping Panda assembly pose adaptation because input frame_id is empty.");
      return;
    }

    geometry_msgs::msg::PoseStamped base_pose;
    if (use_tf_transform_) {
      try {
        base_pose = tf_buffer_->transform(
          input_pose, target_frame_id_, tf2::durationFromSec(tf_lookup_timeout_sec_));
      } catch (const tf2::TransformException & exception) {
        publish_status(
          "tf_lookup_failed", input_pose.header.frame_id, target_frame_id_,
          input_position.x, input_position.y, input_position.z);
        RCLCPP_WARN(
          node_->get_logger(),
          "Failed to transform assembly pose from frame '%s' to '%s': %s",
          input_pose.header.frame_id.c_str(), target_frame_id_.c_str(), exception.what());
        return;
      }
    } else {
      base_pose = input_pose;
      if (!output_frame_id_.empty()) {
        base_pose.header.frame_id = output_frame_id_;
      }
    }

    geometry_msgs::msg::PoseStamped output_pose;
    output_pose.header = base_pose.header;
    output_pose.pose.position.x = base_pose.pose.position.x + x_offset_;
    output_pose.pose.position.y = base_pose.pose.position.y + y_offset_;
    output_pose.pose.position.z = base_pose.pose.position.z + z_offset_;

    if (use_fixed_orientation_) {
      output_pose.pose.orientation.x = fixed_qx_;
      output_pose.pose.orientation.y = fixed_qy_;
      output_pose.pose.orientation.z = fixed_qz_;
      output_pose.pose.orientation.w = fixed_qw_;
    } else {
      output_pose.pose.orientation = base_pose.pose.orientation;
    }

    if (normalize_quaternion_) {
      normalize_orientation(output_pose);
    }

    adapted_pose_publisher_->publish(output_pose);
    publish_status(
      "adapted", input_pose.header.frame_id, output_pose.header.frame_id,
      output_pose.pose.position.x, output_pose.pose.position.y,
      output_pose.pose.position.z);

    RCLCPP_INFO(
      node_->get_logger(),
      "Published Panda assembly pose: input_frame='%s', output_frame='%s', "
      "x=%.3f, y=%.3f, z=%.3f, fixed_orientation=%s",
      input_pose.header.frame_id.c_str(), output_pose.header.frame_id.c_str(),
      output_pose.pose.position.x, output_pose.pose.position.y,
      output_pose.pose.position.z, use_fixed_orientation_ ? "true" : "false");
  }

  void publish_status(
    const std::string & event,
    const std::string & input_frame,
    const std::string & output_frame,
    const double x,
    const double y,
    const double z) const
  {
    std_msgs::msg::String status;
    std::ostringstream stream;
    stream << "event=" << event
           << ";input_frame=" << input_frame
           << ";output_frame=" << output_frame
           << ";use_tf_transform=" << (use_tf_transform_ ? "true" : "false")
           << ";x=" << x
           << ";y=" << y
           << ";z=" << z
           << ";fixed_orientation=" << (use_fixed_orientation_ ? "true" : "false");
    status.data = stream.str();
    status_publisher_->publish(status);
  }

  void normalize_orientation(geometry_msgs::msg::PoseStamped & pose) const
  {
    auto & orientation = pose.pose.orientation;
    const double norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);

    if (norm <= 1e-12) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Output quaternion norm is too small to normalize; using identity orientation.");
      orientation.x = 0.0;
      orientation.y = 0.0;
      orientation.z = 0.0;
      orientation.w = 1.0;
      return;
    }

    orientation.x /= norm;
    orientation.y /= norm;
    orientation.z /= norm;
    orientation.w /= norm;
  }

  rclcpp::Node::SharedPtr node_;
  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  bool use_tf_transform_;
  std::string target_frame_id_;
  double tf_lookup_timeout_sec_;
  std::string status_topic_;
  double x_offset_;
  double y_offset_;
  double z_offset_;
  bool use_fixed_orientation_;
  double fixed_qx_;
  double fixed_qy_;
  double fixed_qz_;
  double fixed_qw_;
  bool normalize_quaternion_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr adapted_pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr assembly_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "panda_assembly_pose_adapter_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto adapter = std::make_shared<PandaAssemblyPoseAdapter>(node);

  rclcpp::spin(node);
  adapter.reset();
  rclcpp::shutdown();
  return 0;
}
