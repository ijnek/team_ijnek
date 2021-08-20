cmake_minimum_required(VERSION 3.5)
project(walk)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nao_command_msgs REQUIRED)
find_package(nao_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(motion_interfaces REQUIRED)
find_package(nao_ik_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

# Build maths_functions lib
add_library(maths_functions SHARED
  src/maths_functions.cpp
)

# Build walk
add_library(walk src/walk.cpp)
ament_target_dependencies(walk
  rclcpp
  nao_sensor_msgs
  geometry_msgs
  nao_ik_interfaces
)
target_link_libraries(walk
  maths_functions)

# Build walk_node
add_executable(walk_node src/walk_node.cpp)
target_link_libraries(walk_node walk)
ament_target_dependencies(walk_node
  rclcpp
  rclcpp_action
  motion_interfaces)

# Install
install(TARGETS
  walk_node
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS maths_functions walk
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
