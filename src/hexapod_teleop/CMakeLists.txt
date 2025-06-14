cmake_minimum_required(VERSION 3.8)
project(hexapod_teleop)

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
find_package(sensor_msgs REQUIRED)
find_package(hexapod_msgs REQUIRED)
find_package(hexapod_common REQUIRED)

# Enable compilation database for Clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


## ════════════════════════════════════════════════════════════════════
##                              Nodes
## ════════════════════════════════════════════════════════════════════


set(EXECUTABLES "")

## ────────────────────────────[ teleop_joy ]──────────────────────────

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


# target_link_libraries(teleop_joy
#   hexapod_common
# )

list(APPEND EXECUTABLES teleop_joy)

# Install executables (nodes)
install(TARGETS
  ${EXECUTABLES}
  DESTINATION lib/${PROJECT_NAME}
)
#
# Install headers so other packages can include them
install(DIRECTORY include/
  DESTINATION include
)

# Export the targets (creates find_package support)
# ament_export_dependencies(rclcpp hexapod_msgs control_msgs)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks (uncomment when ready)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()




