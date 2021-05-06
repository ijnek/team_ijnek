#include "walk_generator/walk_generator.hpp"

#define STAND_HIP_HEIGHT 0.248
#define WALK_HIP_HEIGHT 0.23
#define CROUCH_STAND_PERIOD 0.5

WalkGenerator::WalkGenerator()
    : Node("WalkGenerator")
    , hiph(STAND_HIP_HEIGHT)
{
    sub_twist =
        create_subscription<geometry_msgs::msg::Twist>(
            "turtle1/cmd_vel", 1,
            [this](geometry_msgs::msg::Twist::SharedPtr twist) {
                RCLCPP_INFO(get_logger(), "Recevied twist: %g, %g, %g, %g, %g, %g",
                            twist->linear.x, twist->linear.y, twist->linear.z,
                            twist->angular.x, twist->angular.y, twist->angular.z);
                this->twist = *twist;
            });

    sub_joint_states =
        create_subscription<nao_interfaces::msg::Joints>(
            "sensors/joints", 1,
            [this](nao_interfaces::msg::Joints::SharedPtr sensor_joints) {
                motion_msgs::msg::WalkCommand walkCommand = generate_walk_command(*sensor_joints);
                pub_walk_command->publish(walkCommand);
            });

    pub_walk_command = create_publisher<motion_msgs::msg::WalkCommand>("walk_command", 1);
}

motion_msgs::msg::WalkCommand WalkGenerator::generate_walk_command(nao_interfaces::msg::Joints &sensor_joints)
{
    RCLCPP_INFO(get_logger(), "generate_walk_command called");
    if (twist.angular.z == 0.0)
    {
        if (abs(hiph - STAND_HIP_HEIGHT) < .0001)
        {
            walkOption = STAND;
            t = 0;
        }
        else
        {
            walkOption = STANDUP;
        }
    }
    else
    {
        if (abs(hiph - WALK_HIP_HEIGHT) < .0001)
        {
            walkOption = READY;
            t = 0;
        }
        else
        {
            walkOption = CROUCH;
        }
    }
    RCLCPP_INFO(get_logger(), "walkOption: " + std::to_string(walkOption));

    RCLCPP_INFO(get_logger(), "walkOption: " + std::to_string(walkOption));

    t += dt;

    if (walkOption == STAND)
    {
        hiph = STAND_HIP_HEIGHT;
    }
    else if (walkOption == READY)
    {
        hiph = WALK_HIP_HEIGHT;
    }
    else if (walkOption == CROUCH)
    {
        hiph = STAND_HIP_HEIGHT + (WALK_HIP_HEIGHT - STAND_HIP_HEIGHT) * parabolicStep(t, CROUCH_STAND_PERIOD);
    }
    else if (walkOption == STANDUP)
    {
        hiph = WALK_HIP_HEIGHT + (STAND_HIP_HEIGHT - WALK_HIP_HEIGHT) * parabolicStep(t, CROUCH_STAND_PERIOD);
    }

    motion_msgs::msg::WalkCommand walkCommand;
    walkCommand.hiph = hiph;

    RCLCPP_INFO(get_logger(), "hiph is: %.5f", hiph);

    return walkCommand;
}

float WalkGenerator::parabolicStep(float time, float period, float deadTimeFraction)
{ //normalised [0,1] step up
    float deadTime = period * deadTimeFraction / 2;
    if (time < deadTime + dt / 2)
        return 0;
    if (time > period - deadTime - dt / 2)
        return 1;
    float timeFraction = (time - deadTime) / (period - 2 * deadTime);
    if (time < period / 2)
        return 2.0 * timeFraction * timeFraction;
    return 4 * timeFraction - 2 * timeFraction * timeFraction - 1;
}
