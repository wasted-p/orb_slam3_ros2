cmake_minimum_required(VERSION 3.5)
project(hexapod_description)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# add_custom_target(urdf ALL COMMAND "ros2" "launch" ${PROJECT_NAME} "wamv_description.launch.py")

install(DIRECTORY robots/
  DESTINATION share/${PROJECT_NAME}/robots)


install(DIRECTORY gazebo/
  DESTINATION share/${PROJECT_NAME}/gazebo)

install(DIRECTORY ros2_control/
  DESTINATION share/${PROJECT_NAME}/ros2_control)


install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes)

install(DIRECTORY materials/
  DESTINATION share/${PROJECT_NAME}/materials)


xacro_add_files(
  robots/hexapod.urdf.xacro
  robots/arm.urdf.xacro
    INSTALL DESTINATION robots
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

