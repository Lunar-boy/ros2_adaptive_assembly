#include <chrono>
#include <array>
#include <memory>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{
using Color = std::array<double, 4>;

std::string format_status(
  const std::string & event,
  const std::string & source,
  const geometry_msgs::msg::PoseStamped & pose)
{
  std::ostringstream stream;
  stream << "event=" << event
         << ";source=" << source
         << ";frame=" << pose.header.frame_id
         << ";x=" << pose.pose.position.x
         << ";y=" << pose.pose.position.y
         << ";z=" << pose.pose.position.z;
  return stream.str();
}
}  // namespace

class AdaptiveAssemblyMarkerNode : public rclcpp::Node
{
public:
  AdaptiveAssemblyMarkerNode()
  : Node("adaptive_assembly_marker_node")
  {
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/adaptive_assembly_markers");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/adaptive_assembly_marker_status");
    marker_scale_ = declare_parameter<double>("marker_scale", 0.05);
    arrow_length_ = declare_parameter<double>("arrow_length", 0.12);
    marker_lifetime_sec_ = declare_parameter<double>("marker_lifetime_sec", 0.0);

    if (marker_scale_ <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "marker_scale must be positive; using 0.05 instead of %.3f.",
        marker_scale_);
      marker_scale_ = 0.05;
    }
    if (arrow_length_ <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "arrow_length must be positive; using 0.12 instead of %.3f.",
        arrow_length_);
      arrow_length_ = 0.12;
    }
    if (marker_lifetime_sec_ < 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "marker_lifetime_sec must be non-negative; using 0.0 instead of %.3f.",
        marker_lifetime_sec_);
      marker_lifetime_sec_ = 0.0;
    }

    marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose",
      10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        handle_pose(
          *msg,
          "target_pose",
          visualization_msgs::msg::Marker::SPHERE,
          0,
          Color{0.1, 0.8, 0.1, 0.9});
      });
    pre_grasp_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pre_grasp_pose",
      10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        handle_pose(
          *msg,
          "pre_grasp_pose",
          visualization_msgs::msg::Marker::ARROW,
          1,
          Color{0.1, 0.4, 1.0, 0.9});
      });
    assembly_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/assembly_pose",
      10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        handle_pose(
          *msg,
          "assembly_pose",
          visualization_msgs::msg::Marker::CUBE,
          2,
          Color{1.0, 0.6, 0.1, 0.9});
      });
    panda_pre_grasp_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/panda_pre_grasp_pose",
      10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        handle_pose(
          *msg,
          "panda_pre_grasp_pose",
          visualization_msgs::msg::Marker::ARROW,
          3,
          Color{0.8, 0.1, 1.0, 0.9});
      });

    RCLCPP_INFO(
      get_logger(),
      "Adaptive assembly marker node publishing MarkerArray on '%s' and "
      "status on '%s'. Visualization only: no PlanningScene modification and "
      "no trajectory execution.",
      marker_topic_.c_str(),
      status_topic_.c_str());
  }

private:
  void handle_pose(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & source,
    int marker_type,
    int marker_id,
    const Color & color)
  {
    if (pose.header.frame_id.empty()) {
      publish_status("skipped_empty_frame", source, pose);
      RCLCPP_WARN(
        get_logger(),
        "Skipping marker for %s because incoming pose frame is empty.",
        source.c_str());
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header = pose.header;
    marker.ns = source;
    marker.id = marker_id;
    marker.type = marker_type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose.pose;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);

    if (marker_type == visualization_msgs::msg::Marker::ARROW) {
      marker.scale.x = arrow_length_;
      marker.scale.y = marker_scale_;
      marker.scale.z = marker_scale_;
    } else {
      marker.scale.x = marker_scale_;
      marker.scale.y = marker_scale_;
      marker.scale.z = marker_scale_;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    marker_pub_->publish(marker_array);
    publish_status("marker_updated", source, pose);

    RCLCPP_INFO(
      get_logger(),
      "Updated %s marker in frame '%s' at x=%.3f y=%.3f z=%.3f.",
      source.c_str(),
      pose.header.frame_id.c_str(),
      pose.pose.position.x,
      pose.pose.position.y,
      pose.pose.position.z);
  }

  void publish_status(
    const std::string & event,
    const std::string & source,
    const geometry_msgs::msg::PoseStamped & pose)
  {
    std_msgs::msg::String status;
    status.data = format_status(event, source, pose);
    status_pub_->publish(status);
  }

  std::string marker_topic_;
  std::string status_topic_;
  double marker_scale_{0.05};
  double arrow_length_{0.12};
  double marker_lifetime_sec_{0.0};

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pre_grasp_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr assembly_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    panda_pre_grasp_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdaptiveAssemblyMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
