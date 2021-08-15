cmake_minimum_required(VERSION 3.5)
project(walk_generator)

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
find_package(motion_msgs REQUIRED)

# Build maths_functions lib
add_library(maths_functions SHARED
  src/maths_functions.cpp
)

# Build crouch_generator
add_executable(crouch_generator
  src/crouch_generator_node.cpp
  src/crouch_generator.cpp)

ament_target_dependencies(crouch_generator
  rclcpp
  nao_sensor_msgs
  geometry_msgs
  motion_msgs)

target_link_libraries(
  crouch_generator maths_functions
)

# Build walk_generator
add_executable(walk_generator
  src/walk_generator_node.cpp
  src/walk_generator.cpp)

ament_target_dependencies(walk_generator
  rclcpp
  nao_sensor_msgs
  geometry_msgs
  motion_msgs)

target_link_libraries(
  walk_generator maths_functions
)

# Build inverse_kinematics
add_executable(inverse_kinematics
  src/inverse_kinematics_node.cpp
  src/inverse_kinematics.cpp)

ament_target_dependencies(inverse_kinematics
  rclcpp
  nao_command_msgs
  motion_msgs
  geometry_msgs)

# Install
install(TARGETS
  crouch_generator
  walk_generator
  inverse_kinematics
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/)

install(
  TARGETS maths_functions
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
