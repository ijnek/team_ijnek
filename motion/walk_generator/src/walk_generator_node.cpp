#include "walk_generator/walk_generator.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WalkGenerator>());
    rclcpp::shutdown();
    return 0;
}