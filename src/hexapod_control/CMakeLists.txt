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

add_compile_options(-std=c++11)
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

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks (uncomment when ready)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
