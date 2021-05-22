cmake_minimum_required(VERSION 3.5)
project(balanced_motion_2)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nao_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(motion_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)  

# Build linear library

add_library(linear src/linear.cpp)

ament_target_dependencies(linear
  rclcpp
  Boost
  nao_interfaces
  std_msgs
  ament_index_cpp)

install(
  DIRECTORY pos/
  DESTINATION share/${PROJECT_NAME}/pos/
)

install(TARGETS
  linear
  DESTINATION lib/${PROJECT_NAME}/)

# Build balanced_motion_2
add_executable(balanced_motion_2
  src/balanced_motion_2.cpp)

ament_target_dependencies(balanced_motion_2
  nao_interfaces
  geometry_msgs
  rclcpp
  motion_msgs)

target_link_libraries(balanced_motion_2
  linear)

# Install
install(TARGETS
  balanced_motion_2
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
