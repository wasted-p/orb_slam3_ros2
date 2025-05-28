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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(hexapod_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# Find Eigen
find_package(Eigen3 REQUIRED)

# Enable compilation database for Clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)



# RViz Node
add_executable(hexapod_ik_rviz_node
  src/hexapod_ik_rviz_node.cpp
  src/hexapod_ik_base.cpp
  src/planning_group.cpp
  src/6_dof_marker.cpp
)

# GZ Node
add_executable(hexapod_ik_gz_node
  src/hexapod_ik_gz_node.cpp
  src/hexapod_ik_base.cpp
  src/planning_group.cpp
  src/6_dof_marker.cpp
)


target_include_directories(hexapod_ik_rviz_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(hexapod_ik_gz_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(hexapod_ik_rviz_node
  rclcpp
  std_msgs
  sensor_msgs
  urdf
  kdl_parser
  orocos_kdl
  tf2
  tf2_ros
  hexapod_msgs
  interactive_markers
)

ament_target_dependencies(hexapod_ik_gz_node
  rclcpp
  std_msgs
  trajectory_msgs
  control_msgs
  urdf
  kdl_parser
  orocos_kdl
  tf2
  tf2_ros
  hexapod_msgs
  interactive_markers
)

# Install executables
install(TARGETS  hexapod_ik_rviz_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install executables
install(TARGETS  hexapod_ik_gz_node
  DESTINATION lib/${PROJECT_NAME}
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




