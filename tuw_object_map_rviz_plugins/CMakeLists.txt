cmake_minimum_required(VERSION 3.8)
project(tuw_object_map_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
find_package(yaml_cpp_vendor REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rcpputils)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(tuw_object_map_msgs REQUIRED)

## This setting causes Qt's "MOC" generation to happen automatically, which is required for
## Qt's signal/slot connections to work.
set(CMAKE_AUTOMOC ON)


set(tuw_object_map_rviz_plugins_headers_to_moc
  include/tuw_object_map_rviz_plugins/object_map_display.hpp
)

foreach(header "${tuw_object_map_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(tuw_object_map_rviz_plugins_moc_files "${header}")
endforeach()

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(tuw_object_map_rviz_plugins_source_files
  src/object_map_display.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${tuw_object_map_rviz_plugins_moc_files}
  ${tuw_object_map_rviz_plugins_source_files}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  rviz_common::rviz_common
)

# Causes the visibility macros to use dllexport rather than dllimport (for Windows, when your plugin should be used as library)
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")


ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  rcpputils
  rclcpp
  tuw_object_map_msgs
  rviz_common
  rviz_rendering
  rviz_default_plugins
  yaml_cpp_vendor
)

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rcpputils
  rclcpp
  tuw_object_map_msgs
  rviz_common
  rviz_rendering
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

ament_package()
