cmake_minimum_required(VERSION 3.16)


project(blossom_dxl_hw)

# --- Dependencies (Humble) ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(dynamixel_sdk REQUIRED)  # apt: ros-humble-dynamixel-sdk


include_directories(include)

# --- Hardware plugin (ros2_control SystemInterface) ---

add_library(${PROJECT_NAME} SHARED
  src/blossom_system.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  dynamixel_sdk

)

# Export plugin description so controller_manager can find it

pluginlib_export_plugin_description_file(hardware_interface blossom_dxl_hw_plugins.xml)

# --- C++ tools / nodes ---

add_executable(calibrate_node src/calibrate_node.cpp)

target_compile_features(calibrate_node PUBLIC cxx_std_17)

ament_target_dependencies(calibrate_node
  rclcpp
  dynamixel_sdk
)





# --- Install targets & resources ---
install(TARGETS
  ${PROJECT_NAME}
  calibrate_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  blossom_dxl_hw_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install data (URDF, controllers, launch)

install(DIRECTORY
  urdf
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# --- Python behavior scripts (installed as executables) ---


# Make sure the scripts have a shebang and are chmod +x in your repo; this installs them.


install(PROGRAMS
  src/blossom_yaml_player.py
  src/blossom_behaviors.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- Package export ---

ament_package()