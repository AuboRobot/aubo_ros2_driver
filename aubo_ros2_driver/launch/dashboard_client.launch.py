from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            default_value='127.0.0.1',
            description='JSON-RPC WebSocket server IP'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='9012',
            description='JSON-RPC WebSocket server port'
        ),
        DeclareLaunchArgument(
            'robot',
            default_value='rob1'
        ),
        DeclareLaunchArgument(
            'request_timeout_ms',
            default_value='5000',
            description='JSON-RPC request timeout in milliseconds'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info'
        ),
        Node(
            package='aubo_ros2_driver',
            executable='dashboard_client',
            name='dashboard_client',
            output='screen',
            parameters=[{
                'robot_ip': LaunchConfiguration('robot_ip'),
                'port': LaunchConfiguration('port'),
                'robot': LaunchConfiguration('robot'),
                'request_timeout_ms': LaunchConfiguration('request_timeout_ms'),
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )
    ])
