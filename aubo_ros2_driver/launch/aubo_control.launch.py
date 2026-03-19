from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def _ensure_description_xacro(description_share: str, aubo_type: str) -> None:
    urdf_dir = Path(description_share) / "urdf"
    target = urdf_dir / f"{aubo_type}.urdf.xacro"
    source = urdf_dir / f"{aubo_type}.urdf"

    if target.exists():
        return

    if not source.exists():
        raise FileNotFoundError(
            f"Missing both '{target.name}' and fallback source '{source.name}' in {urdf_dir}"
        )

    lines = source.read_text(encoding="utf-8").splitlines(keepends=True)
    if len(lines) >= 2 and "xmlns:xacro" not in lines[1]:
        lines[1] = lines[1].replace(">", ' xmlns:xacro="http://wiki.ros.org/xacro">', 1)
    lines = [line for line in lines if "<property" not in line]
    target.write_text("".join(lines), encoding="utf-8")


def launch_setup(context, *args, **kwargs):
    del args
    del kwargs

    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package_cfg = LaunchConfiguration("description_package")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    description_file_cfg = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    robot_ip = LaunchConfiguration("robot_ip")
    aubo_type = LaunchConfiguration("aubo_type")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")

    description_package = description_package_cfg.perform(context)
    description_share = get_package_share_directory(description_package)
    _ensure_description_xacro(description_share, aubo_type.perform(context))

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            str(Path(description_share) / "urdf" / "xacro" / "inc" / description_file_cfg.perform(context)),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "aubo_type:=",
            aubo_type,
            " ",
           
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [get_package_share_directory(runtime_config_package.perform(context)), "config", controllers_file]
    )

    rviz_config_file = PathJoinSubstitution(
        [get_package_share_directory(moveit_config_package.perform(context)), "config", "moveit.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    initial_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller,
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    delayed_controller_spawners = RegisterEventHandler(
        OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        joint_state_broadcaster_spawner,
                        initial_joint_controller_spawner,
                    ],
                )
            ],
        )
    )

    return [
        control_node,
        robot_state_publisher_node,
        rviz_node,
        delayed_controller_spawners,
    ]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="aubo_ros2_driver",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "aubo_type",
            default_value="aubo_i5",
            description='Description with aubo robot type.',
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
            "moveit_config_package",
            default_value="aubo_moveit_config",
            description="MoveIt configuration package for the robot, e.g. aubo_moveit_config",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="aubo_ros2.xacro",
            description="URDF/XACRO description file with the robot, e.g. aubo.xacro",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed then also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="None",
            description="IP of robot computer. \
            Used only if 'use_fake_hardware' parameter is false.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "aubo_port",
            default_value="80",
            description="Port at which RWS can be found. \
            Used only if 'use_fake_hardware' parameter is false.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
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
        DeclareLaunchArgument(
            "launch_rviz", default_value="false", description="Launch RViz?"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
