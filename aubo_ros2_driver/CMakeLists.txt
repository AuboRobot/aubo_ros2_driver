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

# 下载 aubo_sdk 发布包(二进制方式引入)
    if(NOT aubo_sdk_POPULATED)
        include(FetchContent)
		if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
			# 使用 ARM64 版本 SDK 的 URL
			set(SDK_URL "https://download.aubo-robotics.cn/sdk/aubo_sdk-0.24.1-rc.3-Linux_aarch64+318754d.zip")
		else()
			# 使用 x86_64 版本 SDK 的 URL
			# set(SDK_URL "https://download.aubo-robotics.cn/sdk/aubo_sdk-0.22.1-Linux_x86_64+2ed0c14.zip")
			set(SDK_URL "https://download.aubo-robotics.cn/sdk/aubo_sdk-0.24.1-rc.3-Linux_x86_64+318754d.zip")
		endif()
		
        FetchContent_Declare(
            aubo_sdk
			URL ${SDK_URL}
        )
        FetchContent_Populate(aubo_sdk)
    endif()
set(aubo_sdk_DIR ${aubo_sdk_SOURCE_DIR}/lib/cmake/aubo_sdk)
message("aubo_sdk_DIR" ${aubo_sdk_SOURCE_DIR}/lib/cmake/aubo_sdk)

message("aubo_sdk_SOURCE_DIR ${aubo_sdk_SOURCE_DIR}")

find_package(aubo_sdk REQUIRED)
find_package(ament_cmake REQUIRED)
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

find_package(Boost REQUIRED)
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

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)
install(
  TARGETS aubo_ros2_control_node 
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
install(PROGRAMS ${aubo_sdk_SOURCE_DIR}/lib/libaubo_sdkd.so
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

ament_package()

