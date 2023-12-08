
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    robot_type = LaunchConfiguration("robot_type")
    
    # General arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf/xacro/inc", description_file]
            ),
            " ",
            "aubo_type:=",
             robot_type,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # There may be other controllers of the joints, but this is the initially-started one
    

    # Gazebo nodes
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
    )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_aubo",
        arguments=["-entity", "aubo", "-topic", "robot_description"],
        output="screen",
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        gazebo,
        gazebo_spawn_robot,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            description="Type/series of used aubo robot.",
            choices=["aubo_i5", "aubo_C3"],
            default_value="aubo_i5",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="aubo_gazebo",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="aubo_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="aubo_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="aubo_ros2.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
   
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
