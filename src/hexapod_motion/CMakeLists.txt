cmake_minimum_required(VERSION 3.8)
project(hexapod_motion)

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
find_package(hexapod_msgs REQUIRED)
find_package(hexapod_control REQUIRED)
find_package(hexapod_common REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)


# Enable compilation database for Clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# Install headers so other packages can include them
install(DIRECTORY include/
  DESTINATION include
)

# Export the targets (creates find_package support)
# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp hexapod_msgs)


# Build motion_server executable
add_executable(motion_server
src/motion_server.cpp
)


ament_target_dependencies(motion_server
  rclcpp
  std_msgs
  hexapod_msgs
  visualization_msgs
  hexapod_control
  hexapod_common
  rclcpp_action
  control_msgs
  tf2
  tf2_ros
  rclcpp_action
  control_msgs
)



# Install headers so other packages can include them
install(DIRECTORY include/
  DESTINATION include
)

# Install executables (nodes)
install(TARGETS
  motion_server
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





