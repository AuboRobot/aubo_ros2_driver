cmake_minimum_required(VERSION 3.22.1)
project(aubo_controllers)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(aubo_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realtime_tools REQUIRED)

add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)

add_library(aubo_controller_plugin SHARED
  src/aubo_joint_trajectory_controller.cpp
  src/aubo_io_status_controller.cpp
  src/freedrive_mode_controller.cpp
)

target_include_directories(aubo_controller_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(aubo_controller_plugin
  aubo_msgs
  controller_interface
  joint_trajectory_controller
  control_msgs
  rclcpp_action
  lifecycle_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  realtime_tools
)

pluginlib_export_plugin_description_file(controller_interface aubo_controller_plugin.xml)

install(
  TARGETS aubo_controller_plugin
  DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_libraries(aubo_controller_plugin)
ament_export_dependencies(
  aubo_msgs
  controller_interface
  joint_trajectory_controller
  control_msgs
  rclcpp_action
  lifecycle_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  realtime_tools
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trajectory_action test/test_trajectory_action.cpp)
  target_include_directories(test_trajectory_action PRIVATE include)
  target_link_libraries(test_trajectory_action aubo_controller_plugin)
  ament_target_dependencies(test_trajectory_action
    joint_trajectory_controller controller_interface control_msgs rclcpp rclcpp_action lifecycle_msgs)
endif()

ament_package()
