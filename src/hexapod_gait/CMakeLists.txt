cmake_minimum_required(VERSION 3.8)
project(hexapod_gait)

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


# Enable compilation database for Clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build hexapod_gait_planner_node executable
add_executable(hexapod_gait_planner_node
src/gait_planner.cpp
)


# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch)
#
# install(DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config)

target_include_directories(hexapod_gait_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)


ament_target_dependencies(hexapod_gait_planner_node
  rclcpp
  std_msgs
  hexapod_msgs
  visualization_msgs
)


# Install executables
install(TARGETS  hexapod_gait_planner_node
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




