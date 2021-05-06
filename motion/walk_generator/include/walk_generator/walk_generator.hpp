#include "rclcpp/rclcpp.hpp"
#include <nao_interfaces/msg/joints.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "motion_msgs/msg/walk_command.hpp"


class WalkGenerator : public rclcpp::Node
{
public:
    WalkGenerator();

private:
    enum WalkOption
    {
        STAND = 0, // with knees straight
        STANDUP = 1, // process of moving from WALK crouch to STAND
        CROUCH = 2, // process of transitioning from STAND to WALK
        READY = 3, // crouch still ready to walk
    };

    WalkOption walkOption = STAND;
    float hiph;
    float dt = 0.02;  // make sure to change this for real robot
    float t = 0.0;

    motion_msgs::msg::WalkCommand generate_walk_command(nao_interfaces::msg::Joints &sensor_joints);
    float parabolicStep(float time, float period, float deadTimeFraction = 0);

    rclcpp::Subscription<nao_interfaces::msg::Joints>::SharedPtr sub_joint_states;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist;
    rclcpp::Publisher<motion_msgs::msg::WalkCommand>::SharedPtr pub_walk_command;

    geometry_msgs::msg::Twist twist;
};