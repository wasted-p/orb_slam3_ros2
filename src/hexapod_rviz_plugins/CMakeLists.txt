cmake_minimum_required(VERSION 3.8)
project(hexapod_rviz_plugins)

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
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hexapod_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
find_package(yaml-cpp REQUIRED)
find_package(hexapod_control REQUIRED)


# QT related setup
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Include directories
include_directories(
  include
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Add the qrc file
qt5_add_resources(RESOURCES_RCC resources.qrc)

# Build the plugin library
add_library(${PROJECT_NAME} SHARED
  src/control/panel.cpp
  include/hexapod_rviz_plugins/control_panel.hpp  # <-- Add this line

  src/motion_editor/panel.cpp
  include/hexapod_rviz_plugins/motion_editor.hpp  # <-- Add this line

  src/motion_editor/pose_list.cpp
  include/ui/pose_list.hpp  # <-- Add this line

  src/joystick_control/panel.cpp
  include/joystick_control/rviz_panel.hpp  # <-- Add this line
  include/joystick_control/joystick.hpp  # <-- Add this line
  src/joystick_control/joystick.cpp
  ${RESOURCES_RCC}  # Add the compiled qrc here
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  trajectory_msgs
  sensor_msgs
  rviz_common
  rviz_rendering
  pluginlib
  hexapod_msgs
  hexapod_control
)

# Export plugin description file
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)


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
