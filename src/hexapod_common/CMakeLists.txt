cmake_minimum_required(VERSION 3.8)
project(hexapod_common)

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
# find_package(urdf REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(hexapod_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp_action REQUIRED)



# ════════════════════════════════════════════════════════════════════
#                          Library
# ════════════════════════════════════════════════════════════════════

add_library(${PROJECT_NAME} SHARED
  src/requests.cpp
  src/yaml_utils.cpp
)



ament_target_dependencies(${PROJECT_NAME}
  ament_cmake
  rclcpp
  # urdf
  std_msgs
  visualization_msgs
  hexapod_msgs
  trajectory_msgs
  control_msgs
  controller_manager_msgs
  rclcpp_action
  yaml-cpp
)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
#
target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)

# Install library
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}

  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # Note: ROS2 convention
)

# Install headers so other packages can include them
install(DIRECTORY include/
  DESTINATION include
)

# Specify include directories for the library
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Export the targets (creates find_package support)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp hexapod_msgs control_msgs rclcpp_action visualization_msgs controller_manager_msgs )

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks (uncomment when ready)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()




