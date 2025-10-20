cmake_minimum_required(VERSION 3.16)
project(blossom_dxl_hw)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/blossom_system.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp hardware_interface pluginlib dynamixel_sdk
)

pluginlib_export_plugin_description_file(hardware_interface blossom_dxl_hw_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES blossom_dxl_hw_plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf config launch DESTINATION share/${PROJECT_NAME})

ament_package()
