cmake_minimum_required(VERSION 3.8)
project(joystick_reader)

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

# Add executable
add_executable(joystick_reader src/joystick_reader.cpp)
target_include_directories(joystick_reader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  joystick_reader
  "rclcpp"
  "sensor_msgs"
  "trajectory_msgs"
  "std_msgs"
)

# Install C++ executables
install(TARGETS joystick_reader
  DESTINATION lib/${PROJECT_NAME})

# Install launch files (if any)
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

ament_package()
