cmake_minimum_required(VERSION 3.5)
project(crouch)

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

# Build crouch
add_library(crouch src/crouch.cpp)
ament_target_dependencies(crouch
  motion_msgs
  std_msgs)

# Build crouch_node
add_executable(crouch_node src/crouch_node.cpp)
target_link_libraries(crouch_node crouch)
ament_target_dependencies(crouch_node
  rclcpp)

# Install
install(TARGETS
  crouch_node
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS crouch
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
