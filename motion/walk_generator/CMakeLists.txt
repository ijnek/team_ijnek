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
find_package(nao_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(motion_msgs REQUIRED)

# Build walk_generator
add_executable(walk_generator 
  src/walk_generator_node.cpp
  src/walk_generator.cpp)

ament_target_dependencies(walk_generator
  rclcpp
  nao_interfaces
  geometry_msgs
  motion_msgs)

# Build inverse_kinematics
add_executable(inverse_kinematics 
  src/inverse_kinematics_node.cpp
  src/inverse_kinematics.cpp)

ament_target_dependencies(inverse_kinematics
  rclcpp
  nao_interfaces
  motion_msgs
  geometry_msgs)

# Install
install(TARGETS
  walk_generator
  inverse_kinematics
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/ 
  DESTINATION share/${PROJECT_NAME}/launch/)

ament_package()
