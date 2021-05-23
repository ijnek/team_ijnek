# Build test_transitioner
ament_add_gtest(test_transitioner
    test_transitioner.cpp)

ament_target_dependencies(test_transitioner
    rclcpp
    nao_interfaces
    geometry_msgs)

target_link_libraries(test_transitioner
    linear
    maths_functions)