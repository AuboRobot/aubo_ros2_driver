from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                description="IP address of the AUBO robot controller",
            ),
            DeclareLaunchArgument("robot_port", default_value="30004"),
            DeclareLaunchArgument("username", default_value="aubo"),
            DeclareLaunchArgument("password", default_value="123456"),
            DeclareLaunchArgument("robot_name", default_value=""),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("mode_timeout_seconds", default_value="60"),
            DeclareLaunchArgument("motion_timeout_seconds", default_value="180"),
            Node(
                package="aubo_ros2_driver",
                executable="aubo_command_node",
                name="aubo_command_node",
                output="screen",
                parameters=[
                    {
                        "robot_ip": LaunchConfiguration("robot_ip"),
                        "robot_port": LaunchConfiguration("robot_port"),
                        "username": LaunchConfiguration("username"),
                        "password": LaunchConfiguration("password"),
                        "robot_name": LaunchConfiguration("robot_name"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "mode_timeout_seconds": LaunchConfiguration(
                            "mode_timeout_seconds"
                        ),
                        "motion_timeout_seconds": LaunchConfiguration(
                            "motion_timeout_seconds"
                        ),
                    }
                ],
            ),
        ]
    )
