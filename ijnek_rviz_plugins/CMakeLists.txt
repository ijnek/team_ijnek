cmake_minimum_required(VERSION 3.8)
project(ijnek_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(ijnek_interfaces REQUIRED)
find_package(pluginlib REQUIRED)

# Build library
set(ijnek_rviz_plugins_headers_to_moc
  include/ijnek_rviz_plugins/field_visualization/field_visualization_display.hpp
)

foreach(header "${ijnek_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(ijnek_rviz_plugins_moc_files "${header}")
endforeach()

set(ijnek_rviz_plugins_source_files
  src/field_visualization/field_visualization_display.cpp
)

add_library(ijnek_rviz_plugins SHARED
  ${ijnek_rviz_plugins_moc_files}
  ${ijnek_rviz_plugins_source_files}
)

target_include_directories(ijnek_rviz_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(ijnek_rviz_plugins PUBLIC
  ${ijnek_interfaces_TARGETS}
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# Export modern CMake targets
ament_export_targets(ijnek_rviz_plugins HAS_LIBRARY_TARGET)

ament_export_dependencies(
  ijnek_interfaces
  rviz_common
  rviz_rendering
)

install(
  TARGETS ijnek_rviz_plugins
  EXPORT ijnek_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
