cmake_minimum_required(VERSION 3.8)
project(hexapod_rviz_panels)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/hexapod_rviz_panels/joint_manager_panel.hpp
)

add_library(hexapod_rviz_panels src/joint_manager_panel.cpp ${MOC_FILES})
target_include_directories(hexapod_rviz_panels PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hexapod_rviz_panels
  pluginlib
  rviz_common
  trajectory_msgs
)
install(TARGETS hexapod_rviz_panels
        EXPORT export_hexapod_rviz_panels
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
install(FILES icons/classes/DemoPanel.png
        DESTINATION share/${PROJECT_NAME}/icons/classes
)
ament_export_include_directories(include)
ament_export_targets(export_hexapod_rviz_panels)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_package()
