cmake_minimum_required(VERSION 3.8)
project(hexapod_teleop)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Enable warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enable compilation database for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hexapod_msgs REQUIRED)
find_package(hexapod_common REQUIRED)  # Your custom library

# === Nodes ===

add_executable(teleop_joy
  src/teleop_joy.cpp
)

ament_target_dependencies(teleop_joy
  rclcpp
  std_msgs
  sensor_msgs
  hexapod_msgs
  hexapod_common
)


# Install targets
install(TARGETS
  teleop_joy
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers (if you have any in this package)
install(DIRECTORY include/
  DESTINATION include
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
