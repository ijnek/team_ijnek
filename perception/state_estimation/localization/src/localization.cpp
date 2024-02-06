#include "localization/localization.hpp"
#include "multi_modal_cmkf.hpp"

namespace localization
{

Localization::Localization(const rclcpp::NodeOptions & options)
: rclcpp::Node{"localization", options}
{
  mmcmkf = std::make_unique<MultiModalCMKF>();

  // Parameters

  // Subscriptions
  sub_markings_ = create_subscription<soccer_vision_3d_msgs::msg::MarkingArray>(
    "soccer_vision_3d/markings", 1,
    std::bind(&Localization::markings_callback, this, std::placeholders::_1));

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", 1, std::bind(&Localization::odom_callback, this, std::placeholders::_1));

  // Publishers
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void Localization::markings_callback(const soccer_vision_3d_msgs::msg::MarkingArray::SharedPtr msg)
{
  mmcmkf->update(*msg);
}

void Localization::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  mmcmkf->predict(*msg);
}


}  // namespace localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization::Localization)
