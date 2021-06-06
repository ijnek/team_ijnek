cmake_minimum_required(VERSION 3.5)
project(kick)

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
find_package(nao_interfaces REQUIRED)

ament_export_include_directories(include)

# Build kick
add_library(kick src/kick.cpp)
ament_target_dependencies(kick
  motion_msgs
  std_msgs
  nao_interfaces)

# Build kick_node
add_executable(kick_node src/kick_node.cpp)
target_link_libraries(kick_node kick)
ament_target_dependencies(kick_node
  rclcpp)

# Install
install(TARGETS
  kick_node
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS kick
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
