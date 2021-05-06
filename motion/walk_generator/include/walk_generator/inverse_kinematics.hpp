#include "rclcpp/rclcpp.hpp"
#include "motion_msgs/msg/walk_command.hpp"
#include <nao_interfaces/msg/joints.hpp>
#include <geometry_msgs/msg/point.hpp>

// Use for iterative inverse kinematics for turning (see documentation BH 2010)
struct Hpr
{
    float Hp;
    float Hr;
};

class InverseKinematics : public rclcpp::Node
{
public:
    InverseKinematics();

private:
    nao_interfaces::msg::Joints calculate_joints(motion_msgs::msg::WalkCommand &walk_command);
    geometry_msgs::msg::Point mf2b(float Hyp, float Hp, float Hr, float Kp, float Ap, float Ar, float xf, float yf, float zf);
    Hpr hipAngles(float Hyp, float Hp, float Hr, float Kp, float Ap,
                 float Ar, float xf, float yf, float zf, geometry_msgs::msg::Point e);

    rclcpp::Subscription<motion_msgs::msg::WalkCommand>::SharedPtr sub_walk_command;
    rclcpp::Publisher<nao_interfaces::msg::Joints>::SharedPtr pub_joints;
};