cmake_minimum_required(VERSION 3.8)
project(hexapod_control)

# Set C++ standard to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Enable compiler warnings for GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

# Find Eigen
find_package(Eigen3 REQUIRED)

# Enable compilation database for Clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# Build leg_control_node executable
add_executable(leg_control_node
src/node/leg_control.cpp
src/node/marker.cpp
)


install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

target_include_directories(leg_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(leg_control_node
  rclcpp
  std_msgs
  visualization_msgs
  interactive_markers
  urdf
  kdl_parser
  orocos_kdl
)
target_link_libraries(leg_control_node Eigen3::Eigen)

# Install executables
install(TARGETS  leg_control_node
  DESTINATION lib/${PROJECT_NAME}
)


# Library

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
  src/rviz/panel.cpp
  include/rviz/panel.hpp  # <-- Add this line
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
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
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

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks (uncomment when ready)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()




