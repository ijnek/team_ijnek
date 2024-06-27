#include "localization/mmcmkf.hpp"
#include "multi_modal_cmkf.hpp"

using namespace std::chrono_literals;

namespace localization
{

MMCMKF::MMCMKF(const rclcpp::NodeOptions & options)
: rclcpp::Node{"mmcmkf", options}
{
  mmcmkf = std::make_unique<MultiModalCMKF>();

  // Parameters

  // Subscriptions
  sub_markings_ = create_subscription<soccer_vision_3d_msgs::msg::MarkingArray>(
    "marking_array", 1,
    std::bind(&MMCMKF::markings_callback, this, std::placeholders::_1));

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", 1, std::bind(&MMCMKF::odom_callback, this, std::placeholders::_1));

  // Services
  set_poses_srv_ = create_service<ijnek_interfaces::srv::SetPoses>(
    "set_poses", std::bind(&MMCMKF::set_poses_cb, this, std::placeholders::_1, std::placeholders::_2));

  // Publishers
  pub_pwcs_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pwcs", 1);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Timer
  timer_ = create_wall_timer(100ms, std::bind(&MMCMKF::timer_callback, this));
}

void MMCMKF::markings_callback(const soccer_vision_3d_msgs::msg::MarkingArray::SharedPtr msg)
{
  mmcmkf->update(*msg);
}

void MMCMKF::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  mmcmkf->predict(*msg);
}

void MMCMKF::set_poses_cb(
  const std::shared_ptr<ijnek_interfaces::srv::SetPoses::Request> request,
  std::shared_ptr<ijnek_interfaces::srv::SetPoses::Response> response)
{
  (void) request;
  (void) response;
  RCLCPP_INFO(get_logger(), "Received request to set poses!");
}

void MMCMKF::timer_callback()
{
  auto pose_with_covariance = mmcmkf->getPoseWithCovariance();

  // Publish pose with covariance
  geometry_msgs::msg::PoseWithCovarianceStamped pwcs;
  pwcs.header.stamp = now();
  pwcs.header.frame_id = "map";
  pwcs.pose = pose_with_covariance;
  pub_pwcs_->publish(pwcs);

  // Publish tf
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now();
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = pose_with_covariance.pose.position.x;
  transform.transform.translation.y = pose_with_covariance.pose.position.y;
  transform.transform.translation.z = pose_with_covariance.pose.position.z;
  transform.transform.rotation = pose_with_covariance.pose.orientation;
  tf_broadcaster_->sendTransform(transform);
}

}  // namespace localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::MMCMKF)
