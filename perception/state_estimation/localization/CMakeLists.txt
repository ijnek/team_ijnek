cmake_minimum_required(VERSION 3.5)
project(localization)

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

# Find packages
find_package(ament_cmake REQUIRED)
find_package(game_controller_spl_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ijnek_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(soccer_vision_3d_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  game_controller_spl_interfaces
  Eigen3
  ijnek_interfaces
  nav_msgs
  rclcpp
  rclcpp_components
  soccer_vision_3d_msgs
  tf2_ros)

# Build mmcmkf node
add_library(mmcmkf_node SHARED
  src/cmkf.cpp
  src/mmcmkf.cpp
  src/multi_modal_cmkf.cpp)

target_include_directories(mmcmkf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR})

rclcpp_components_register_node(mmcmkf_node
  PLUGIN "localization::MMCMKF"
  EXECUTABLE mmcmkf)

ament_target_dependencies(mmcmkf_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Build transition_detector node
add_library(transition_detector_node SHARED
  src/transition_detector.cpp)

target_include_directories(transition_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR})

rclcpp_components_register_node(transition_detector_node
  PLUGIN "localization::TransitionDetector"
  EXECUTABLE transition_detector)

ament_target_dependencies(transition_detector_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Build pose_resetter node
add_library(pose_resetter_node SHARED
  src/pose_resetter.cpp)

target_include_directories(pose_resetter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR})

rclcpp_components_register_node(pose_resetter_node
  PLUGIN "localization::PoseResetter"
  EXECUTABLE pose_resetter)

ament_target_dependencies(pose_resetter_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Ament export
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS mmcmkf_node transition_detector_node pose_resetter_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_resetter test/test_pose_resetter.cpp)
  target_link_libraries(test_pose_resetter pose_resetter_node)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
