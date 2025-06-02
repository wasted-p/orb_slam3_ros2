cmake_minimum_required(VERSION 3.5)
project(hexapod_sim)

# === Set Ignition/Gazebo (Gazebo Sim) version variables ===
set(GZ_COMMON_VER 5)
set(GZ_PLUGIN_VER 2)
set(GZ_MATH_VER 7)
set(GZ_TRANSPORT_VER 13)
set(GZ_SIM_VER 8)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# === Required for ROS 2 ===
find_package(ament_cmake REQUIRED)

# find dependencies
find_package(xacro REQUIRED)

# === Find gz packages ===
find_package(gz-common${GZ_COMMON_VER} REQUIRED)
find_package(gz-plugin${GZ_PLUGIN_VER} REQUIRED COMPONENTS register)
find_package(gz-math${GZ_MATH_VER} REQUIRED)
find_package(gz-transport${GZ_TRANSPORT_VER} REQUIRED)
find_package(gz-sim${GZ_SIM_VER} REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# === Add include directory for headers ===
include_directories(include)

# === Define plugin ===
add_library(ResetListenerSystem SHARED src/ResetListenerSystem.cpp)

# C++ standard
set_property(TARGET ResetListenerSystem PROPERTY CXX_STANDARD 17)

# Add include dirs for build and install
target_include_directories(ResetListenerSystem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link against GZ libraries
target_link_libraries(ResetListenerSystem
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  # gz-msgs10::gz-msgs10
)


ament_target_dependencies(ResetListenerSystem
  rclcpp
  std_msgs

)

# === Install target and headers ===
install(TARGETS ResetListenerSystem
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)
# Important: Set symbol visibility
set_target_properties(ResetListenerSystem PROPERTIES
  CXX_VISIBILITY_PRESET default
  VISIBILITY_INLINES_HIDDEN OFF
)

install(
  DIRECTORY  worlds models config rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
