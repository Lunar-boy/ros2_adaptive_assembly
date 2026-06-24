#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class PandaPreGraspPoseAdapter
{
public:
  explicit PandaPreGraspPoseAdapter(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    input_topic_(declare_parameter<std::string>("input_topic", "/pre_grasp_pose")),
    output_topic_(declare_parameter<std::string>("output_topic", "/panda_pre_grasp_pose")),
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
    adapted_pose_publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      output_topic_, 10);
    pre_grasp_subscription_ =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        pre_grasp_pose_callback(*message);
      });

    RCLCPP_INFO(
      node_->get_logger(),
      "Panda pre-grasp pose adapter ready: input_topic='%s', output_topic='%s', "
      "use_fixed_orientation=%s.",
      input_topic_.c_str(), output_topic_.c_str(),
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

  void pre_grasp_pose_callback(const geometry_msgs::msg::PoseStamped & input_pose)
  {
    const auto & input_position = input_pose.pose.position;
    RCLCPP_INFO(
      node_->get_logger(),
      "Received task pre-grasp pose: frame='%s', x=%.3f, y=%.3f, z=%.3f",
      input_pose.header.frame_id.c_str(), input_position.x, input_position.y,
      input_position.z);

    geometry_msgs::msg::PoseStamped output_pose;
    output_pose.header = input_pose.header;
    output_pose.pose.position.x = input_pose.pose.position.x + x_offset_;
    output_pose.pose.position.y = input_pose.pose.position.y + y_offset_;
    output_pose.pose.position.z = input_pose.pose.position.z + z_offset_;

    if (use_fixed_orientation_) {
      output_pose.pose.orientation.x = fixed_qx_;
      output_pose.pose.orientation.y = fixed_qy_;
      output_pose.pose.orientation.z = fixed_qz_;
      output_pose.pose.orientation.w = fixed_qw_;
    } else {
      output_pose.pose.orientation = input_pose.pose.orientation;
    }

    if (normalize_quaternion_) {
      normalize_orientation(output_pose);
    }

    adapted_pose_publisher_->publish(output_pose);

    const auto & output_position = output_pose.pose.position;
    const auto & output_orientation = output_pose.pose.orientation;
    RCLCPP_INFO(
      node_->get_logger(),
      "Published Panda pre-grasp pose: frame='%s', x=%.3f, y=%.3f, z=%.3f, "
      "qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f, fixed_orientation=%s",
      output_pose.header.frame_id.c_str(), output_position.x, output_position.y,
      output_position.z, output_orientation.x, output_orientation.y,
      output_orientation.z, output_orientation.w,
      use_fixed_orientation_ ? "true" : "false");
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
        "Output quaternion norm is too small to normalize; using identity "
        "orientation.");
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
  double x_offset_;
  double y_offset_;
  double z_offset_;
  bool use_fixed_orientation_;
  double fixed_qx_;
  double fixed_qy_;
  double fixed_qz_;
  double fixed_qw_;
  bool normalize_quaternion_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr adapted_pose_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pre_grasp_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "panda_pre_grasp_pose_adapter_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto adapter = std::make_shared<PandaPreGraspPoseAdapter>(node);

  rclcpp::spin(node);
  adapter.reset();
  rclcpp::shutdown();
  return 0;
}
