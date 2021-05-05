#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class StaticPosePublisher : public rclcpp::Node
{
public:
    StaticPosePublisher() : Node("StaticPosePublisher")
    {
        this->declare_parameter("initial_pose_x", 0.0);
        this->declare_parameter("initial_pose_y", 0.0);
        this->declare_parameter("initial_pose_theta", 0.0);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&StaticPosePublisher::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = get_parameter("initial_pose_x").as_double();
        transformStamped.transform.translation.y = get_parameter("initial_pose_y").as_double();
        transformStamped.transform.translation.z = get_parameter("initial_pose_theta").as_double();
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(transformStamped);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticPosePublisher>());
    rclcpp::shutdown();
    return 0;
}