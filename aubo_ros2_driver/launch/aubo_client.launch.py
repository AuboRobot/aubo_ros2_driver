from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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
            'log_level', 
            default_value='info'
        ),
        Node(
            package='aubo_ros2_driver',
            executable='aubo_client_node.py',
            name='aubo_client',
            output='screen',
            parameters=[{
                'jsonrpc.ip': LaunchConfiguration('robot_ip'),
                'jsonrpc.port': LaunchConfiguration('port'),
                'jsonrpc.robot_prefix': LaunchConfiguration('robot'),
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )
    ])
