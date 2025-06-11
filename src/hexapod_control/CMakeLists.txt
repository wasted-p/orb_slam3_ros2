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
find_package(controller_manager_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)


# Enable compilation database for Clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


set(EXECUTABLES "")

# JointStatePublisher
# ====================
add_executable(joint_state_publisher
  src/joint_state_publisher.cpp
)

ament_target_dependencies(joint_state_publisher
  rclcpp
  std_msgs
  sensor_msgs
  hexapod_msgs
)

target_link_libraries(joint_state_publisher
  yaml-cpp
)

list(APPEND EXECUTABLES joint_state_publisher)

# KinematicsService
# ====================
add_executable(kinematics_service
  src/kinematics_service.cpp
)

ament_target_dependencies(kinematics_service
  rclcpp
  std_msgs
  sensor_msgs
  urdf
  kdl_parser
  orocos_kdl
  hexapod_msgs
)

list(APPEND EXECUTABLES kinematics_service)

# PosePublisher
# ====================
add_executable(pose_publisher
  src/pose_publisher.cpp
)

ament_target_dependencies(pose_publisher
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  hexapod_msgs
)

list(APPEND EXECUTABLES pose_publisher)

# TrajectoryPublisher
# ====================
add_executable(trajectory_publisher
  src/trajectory_publisher.cpp
)

ament_target_dependencies(trajectory_publisher
  rclcpp
  std_msgs
  trajectory_msgs
  rclcpp_action
  control_msgs
  hexapod_msgs
  controller_manager_msgs
)

list(APPEND EXECUTABLES trajectory_publisher)

# VisualizationServer
# ====================
add_executable(visualization_server
  src/visualization_server.cpp
  src/6_dof_marker.cpp
)

ament_target_dependencies(visualization_server
  rclcpp
  std_msgs
  geometry_msgs
  hexapod_msgs
  visualization_msgs
  interactive_markers
)

list(APPEND EXECUTABLES visualization_server)

# ShutdownControllers
# ====================
add_executable(shutdown_controllers
  src/shutdown_controllers.cpp
)

ament_target_dependencies(shutdown_controllers
  rclcpp
  std_msgs
  controller_manager_msgs
)

list(APPEND EXECUTABLES shutdown_controllers)

# StartupControllers
# ====================
add_executable(startup_controllers
  src/startup_controllers.cpp
)

ament_target_dependencies(startup_controllers
  rclcpp
  std_msgs
  controller_manager_msgs
)

list(APPEND EXECUTABLES shutdown_controllers)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
#
# target_include_directories(hexapod_trajectory_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )
#
# target_include_directories(kinematics_service PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )
#


# Install executables
install(TARGETS  ${EXECUTABLES}
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




