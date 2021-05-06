#include "walk_generator/inverse_kinematics.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InverseKinematics>());
    rclcpp::shutdown();
    return 0;
}