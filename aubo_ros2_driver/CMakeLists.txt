cmake_minimum_required(VERSION 3.22.1)
set(CMAKE_CXX_STANDARD 17)
cmake_policy(SET CMP0097 NEW)
project(aubo_ros2_driver)

add_definitions( -DROS_BUILD )

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(ENV{CPM_SOURCE_CACHE} $ENV{HOME}/.cache/CPM)

# aubo_sdk: prefer AUBO_SDK_ROOT (pre-extracted), else FetchContent zip
if(DEFINED ENV{AUBO_SDK_ROOT} AND EXISTS "$ENV{AUBO_SDK_ROOT}/lib/cmake/aubo_sdk")
  set(aubo_sdk_SOURCE_DIR "$ENV{AUBO_SDK_ROOT}")
  set(aubo_sdk_POPULATED TRUE)
  message(STATUS "${PROJECT_NAME}: using AUBO_SDK_ROOT=${aubo_sdk_SOURCE_DIR}")
elseif(NOT aubo_sdk_POPULATED)
  include(FetchContent)
  if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    message(STATUS "${PROJECT_NAME}: FetchContent aubo_sdk aarch64")
    set(SDK_URL "https://download.aubo-robotics.cn/sdk/aubo_sdk-0.27.0-rc.4-Linux_aarch64+b7ed04d.zip")
  else()
    message(STATUS "${PROJECT_NAME}: FetchContent aubo_sdk x86_64")
    set(SDK_URL "https://download.aubo-robotics.cn/sdk/aubo_sdk-0.27.0-rc.4-Linux_x86_64+b7ed04d.zip")
  endif()
  FetchContent_Declare(aubo_sdk URL ${SDK_URL})
  FetchContent_Populate(aubo_sdk)
endif()
set(aubo_sdk_DIR ${aubo_sdk_SOURCE_DIR}/lib/cmake/aubo_sdk)
message(STATUS "aubo_sdk_DIR=${aubo_sdk_DIR}")
message(STATUS "aubo_sdk_SOURCE_DIR=${aubo_sdk_SOURCE_DIR}")

find_package(aubo_sdk REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(aubo_dashboard_msgs REQUIRED)
find_package(aubo_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  controller_manager_msgs
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  tf2_geometry_msgs
  aubo_msgs
  aubo_dashboard_msgs
)

add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)


include_directories(
  include
  
  ${Boost_INCLUDE_DIRS}
  ${aubo_sdk_SOURCE_DIR}/include
  
)

install(PROGRAMS
  scripts/test_aubo_jsonrpc.py
  DESTINATION lib/${PROJECT_NAME}
)

add_library(aubo_robot_driver_plugin 
  SHARED
  src/aubo_hardware_interface.cpp
)
target_link_libraries(aubo_robot_driver_plugin  aubo_sdk::aubo_sdk)
ament_target_dependencies(aubo_robot_driver_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(
  aubo_robot_driver_plugin
  PRIVATE
  include
)

add_executable(aubo_ros2_control_node
  src/aubo_ros2_control_node.cpp
)

ament_target_dependencies(aubo_ros2_control_node 
    controller_manager 
    rclcpp)

add_executable(aubo_command_node
  src/aubo_command_node.cpp
)
target_link_libraries(aubo_command_node aubo_sdk::aubo_sdk Threads::Threads)
ament_target_dependencies(aubo_command_node
  aubo_msgs
  geometry_msgs
  rclcpp
)
add_executable(robot_state_helper
  src/robot_state_helper.cpp
  src/robot_state_helper_node.cpp
)
ament_target_dependencies(robot_state_helper ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(
  robot_state_helper
  PRIVATE
  include
)

add_executable(controller_stopper
  src/controller_stopper.cpp
  src/controller_stopper_node.cpp
)
ament_target_dependencies(controller_stopper ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(
  controller_stopper
  PRIVATE
  include
)

add_executable(dashboard_client
  src/dashboard_client_ros.cpp
  src/dashboard_client_node.cpp
)
ament_target_dependencies(dashboard_client ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(dashboard_client Boost::system nlohmann_json::nlohmann_json)
target_include_directories(
  dashboard_client
  PRIVATE
  include
)

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)
install(
  TARGETS aubo_ros2_control_node aubo_command_node robot_state_helper controller_stopper dashboard_client
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS aubo_robot_driver_plugin
  DESTINATION lib
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  aubo_robot_driver_plugin
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES
  ${aubo_sdk_SOURCE_DIR}/lib/libaubo_sdk.so
  ${aubo_sdk_SOURCE_DIR}/lib/libaubo_sdkd.so
  DESTINATION lib
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
ament_python_install_package(${PROJECT_NAME})

#install(DIRECTORY config launch
#  DESTINATION share/${PROJECT_NAME}
#)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_control_profile COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_control_profile.py)
  ament_add_gtest(test_servo_return_policy test/test_servo_return_policy.cpp)
  target_include_directories(test_servo_return_policy PRIVATE include)
endif()

ament_package()
