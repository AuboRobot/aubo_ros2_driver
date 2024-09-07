# Copyright 2024 AUBO Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch_ros.parameter_descriptions import ParameterFile
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'runtime_config_package',
            default_value='aubo_description',
            description='Package with the controller\'s configuration in "config" folder. \
                         Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'controllers_file',
            default_value='aubo_controllers.yaml',
            description='YAML file with the controllers configuration.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='aubo_description',
            description='Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='aubo.urdf.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'tf_prefix',
            default_value="",
            description='Prefix of the joint names, useful for multi-robot setup. \
                         If changed than also joint names in the controllers \
                         configuration have to be updated. Expected format "<tf_prefix>/"',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='/',
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Start robot with fake hardware mirroring command to its states.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_planning',
            default_value='false',
            description='Start robot with Moveit2 `move_group` planning \
                         config for Pilz and OMPL.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_servoing',
            default_value='false',
            description='Start robot with Moveit2 servoing.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_controller',
            default_value='joint_trajectory_controller',
            description='Robot controller to start.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            default_value='127.0.0.1',
            description='Robot IP',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rtu_device_name',
            default_value='/dev/ttyS7,115200,N,8,1',
            description='Modbus RTU device info',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "aubo_type",
            default_value="aubo_ES3",
            description='Description with aubo robot type.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'initial_positions_file',
            default_value='initial_positions.yaml',
            description='Configuration file of robot initial positions for simulation.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'base_frame_file',
            default_value='base_frame.yaml',
            description='Configuration file of robot base frame wrt World.',
        )
    )

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    controllers_file = LaunchConfiguration('controllers_file')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    tf_prefix = LaunchConfiguration('tf_prefix')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_planning = LaunchConfiguration('use_planning')
    use_servoing = LaunchConfiguration('use_servoing')
    robot_controller = LaunchConfiguration('robot_controller')
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    launch_rviz = LaunchConfiguration('launch_rviz')
    robot_ip = LaunchConfiguration('robot_ip')
    rtu_device_name = LaunchConfiguration('rtu_device_name')
    aubo_type = LaunchConfiguration("aubo_type")
    initial_positions_file = LaunchConfiguration('initial_positions_file')
    base_frame_file = LaunchConfiguration('base_frame_file')
    namespace = LaunchConfiguration('namespace')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'urdf', description_file]
            ),
            ' ',
            'tf_prefix:=',
            tf_prefix,
            ' ',
            'use_fake_hardware:=',
            use_fake_hardware,
            ' ',
            'robot_ip:=',
            robot_ip,
            ' ',
            'rtu_device_name:=',
            rtu_device_name,
            ' ',
            'aubo_type:=',
            aubo_type,
            ' ',
            'initial_positions_file:=',
            PathJoinSubstitution(
                [FindPackageShare(runtime_config_package), 'config', initial_positions_file]
            ),
            ' ',
            'base_frame_file:=',
            PathJoinSubstitution(
                [FindPackageShare(runtime_config_package), 'config', base_frame_file]
            ),
            ' ',
            'namespace:=',
            namespace,
        ]
    )

    robot_description = {'robot_description': robot_description_content}
    
    # Running with Moveit2 planning
    aubo_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('aubo_bringup'),
            '/launch',
            '/aubo_planning.launch.py'
        ]),
        launch_arguments={
            'description_package': description_package,
            'description_file': description_file,
            'tf_prefix': tf_prefix,
            'rtu_device_name': rtu_device_name,
            'launch_rviz': launch_rviz,
            'base_frame_file': PathJoinSubstitution(
                [FindPackageShare(runtime_config_package), 'config', base_frame_file]
            ),
            'namespace': namespace,
            'aubo_type': aubo_type,
        }.items(),
        condition=IfCondition(use_planning),
    )

    # Running with Moveit2 servoing
    aubo_servoing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('aubo_bringup'),
            '/launch',
            '/aubo_servoing.launch.py'
        ]),
        launch_arguments={
            'description_package': description_package,
            'description_file': description_file,
            'tf_prefix': tf_prefix,
            'rtu_device_name': rtu_device_name,
            'base_frame_file': PathJoinSubstitution(
                [FindPackageShare(runtime_config_package), 'config', base_frame_file]
            ),
            'namespace': namespace,
            'aubo_type': aubo_type,
        }.items(),
        condition=IfCondition(use_servoing),
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            'config',
            controllers_file,
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), 'rviz', 'aubo.rviz']
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(robot_controllers, allow_substs=True),
        ],
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )

    aubo_control_node = Node(
        package='aubo_hardware',
        executable='aubo_ros2_control_node',
        parameters=[robot_description, 
                    ParameterFile(robot_controllers, allow_substs=True),
                    ],
        output='both',
        namespace=namespace,
        condition=UnlessCondition(use_fake_hardware),
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
        ],
        condition=UnlessCondition(use_planning),
    )


    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager',
                   [namespace, 'controller_manager']],
    )

    # Spawn controllers
    def controller_spawner(name, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                name,
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags,
            condition=UnlessCondition(use_fake_hardware),
        )
    controller_spawner_names = [
        "io_and_status_controller",
    ]
    controller_spawner_inactive_names = ["forward_command_controller_position"]

    # robot_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=[robot_controller, '--controller-manager', [namespace, 'controller_manager']],
    # )
    gpio_controller_spawner = [controller_spawner(name) for name in controller_spawner_names] + [
        controller_spawner(name, active=False) for name in controller_spawner_inactive_names
    ]
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            robot_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
        ],
    )


    # Delay `joint_state_broadcaster` after control_node
    delay_joint_state_broadcaster_spawner_after_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_broadcaster_spawner],
        ),
        condition=IfCondition(use_fake_hardware),
    )
    delay_joint_state_broadcaster_spawner_after_aubo_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=aubo_control_node,
            on_start=[joint_state_broadcaster_spawner],
        ),
        condition=UnlessCondition(use_fake_hardware),
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )
    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_joint_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        aubo_control_node,
        robot_state_pub_node,
        aubo_planning_launch,
        aubo_servoing_launch,
        delay_joint_state_broadcaster_spawner_after_control_node,
        delay_joint_state_broadcaster_spawner_after_aubo_control_node,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_controller_spawner_after_joint_state_broadcaster_spawner,
    ] + gpio_controller_spawner

    return LaunchDescription(declared_arguments + nodes)
