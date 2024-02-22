#pragma once

#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "soccer_vision_3d_msgs/msg/marking_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "ijnek_interfaces/srv/set_poses.hpp"

namespace localization
{

// Forward declaration
class MultiModalCMKF;

class MMCMKF : public rclcpp::Node
{
public:
  explicit MMCMKF(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:

  void markings_callback(const soccer_vision_3d_msgs::msg::MarkingArray::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<soccer_vision_3d_msgs::msg::MarkingArray>::SharedPtr sub_markings_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Reset poses service
  rclcpp::Service<ijnek_interfaces::srv::SetPoses>::SharedPtr set_poses_srv_;
  void set_poses_cb(
    const std::shared_ptr<ijnek_interfaces::srv::SetPoses::Request> request,
    std::shared_ptr<ijnek_interfaces::srv::SetPoses::Response> response);

  std::unique_ptr<MultiModalCMKF> mmcmkf;
};

}  // namespace localization
