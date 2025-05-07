cmake_minimum_required(VERSION 3.8)
project(joystick_control)

# Enable compilation database for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

# QT related setup
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Include directories
include_directories(
  include
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Build the plugin library
add_library(${PROJECT_NAME} SHARED
  src/rviz_panel.cpp
  include/joystick_control/rviz_panel.hpp  # <-- Add this line
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  rviz_common
  rviz_rendering
  pluginlib
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)


# Plugin export
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(
  TARGETS joystick_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers (if you have a public API)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(joystick_control)
ament_export_dependencies(
  rclcpp
  std_msgs
  rviz_common
  rviz_rendering
  pluginlib
)

# Install
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install launch files (if any)
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  rviz_common
  rviz_rendering
  pluginlib
  Qt5
)
ament_package()
