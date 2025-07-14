cmake_minimum_required(VERSION 3.10)
project(xl330_ros2_control)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(urdf REQUIRED)
find_package(ros2_control REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/xl330_hardware_interface.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  rclcpp
  pluginlib
  dynamixel_sdk
)

pluginlib_export_plugin_description_file(hardware_interface xl330_hardware_interface_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config launch description
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/teleop_xl330.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()