cmake_minimum_required(VERSION 3.5)
project(motion_transitioner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(motion_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_export_include_directories(include)

# Build motion_transitioner
add_library(motion_transitioner src/motion_transitioner.cpp)
ament_target_dependencies(motion_transitioner
  motion_msgs
  std_msgs)

# Build motion_transitioner_node
add_executable(motion_transitioner_node src/motion_transitioner_node.cpp)
target_link_libraries(motion_transitioner_node motion_transitioner)
ament_target_dependencies(motion_transitioner_node
  rclcpp)

# Install
install(TARGETS
  motion_transitioner_node
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS motion_transitioner
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
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
