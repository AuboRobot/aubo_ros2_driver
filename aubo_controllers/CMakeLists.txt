cmake_minimum_required(VERSION 3.22.1)
project(aubo_controllers)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(aubo_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)

add_library(aubo_controller_plugin SHARED
  src/aubo_io_status_controller.cpp
  src/freedrive_mode_controller.cpp
)

ament_target_dependencies(aubo_controller_plugin
  aubo_msgs
  controller_interface
  lifecycle_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

pluginlib_export_plugin_description_file(controller_interface aubo_controller_plugin.xml)

install(
  TARGETS aubo_controller_plugin
  DESTINATION lib
)

ament_export_libraries(aubo_controller_plugin)
ament_export_dependencies(
  aubo_msgs
  controller_interface
  lifecycle_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

ament_package()
