# Build test_transitioner
ament_add_gtest(test_transitioner
    test_transitioner.cpp)

ament_target_dependencies(test_transitioner
    nao_interfaces
    geometry_msgs)