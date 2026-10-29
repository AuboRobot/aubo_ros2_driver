from pathlib import Path
import tempfile
import yaml
from aubo_ros2_driver.control_profile import resolve_update_rate, resolve_servo_period

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction, EmitEvent, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def _ensure_description_xacro(description_share: str, aubo_type: str) -> None:
    urdf_dir = Path(description_share) / "urdf"
    target = urdf_dir / f"{aubo_type}.urdf.xacro"
    source = urdf_dir / f"{aubo_type}.urdf"

    if aubo_type.endswith("_calibrated") and not source.is_file():
        raise FileNotFoundError(f"Calibrated URDF missing: {source}. Generate it and rebuild aubo_description first.")

    if target.exists() and (not source.exists() or target.stat().st_mtime >= source.stat().st_mtime):
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
    servo_mode = LaunchConfiguration("servo_mode")
    dashboard_port = LaunchConfiguration("dashboard_port")
    dashboard_robot = LaunchConfiguration("dashboard_robot")
    aubo_type = LaunchConfiguration("aubo_type").perform(context)
    use_calibrated = LaunchConfiguration("use_calibrated").perform(context).lower()
    if use_calibrated not in ("true", "false"):
        raise ValueError("use_calibrated must be true or false")
    if use_calibrated == "true":
        aubo_type += "_calibrated"
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    remote_viz = LaunchConfiguration("remote_viz")
    # remote_viz:=true → 不启本机 RViz（对齐 dual 多机）
    effective_launch_rviz = PythonExpression(
        ["'false' if '", remote_viz, "' == 'true' else '", launch_rviz, "'"]
    )

    description_package = description_package_cfg.perform(context)
    description_share = get_package_share_directory(description_package)
    _ensure_description_xacro(description_share, aubo_type)

    controller_path = Path(get_package_share_directory(runtime_config_package.perform(context))) / "config" / controllers_file.perform(context)
    config = yaml.safe_load(controller_path.read_text())
    rate = resolve_update_rate(aubo_type,
        LaunchConfiguration("controller_profile").perform(context),
        LaunchConfiguration("update_rate").perform(context),
        config["controller_manager"]["ros__parameters"]["update_rate"])
    servo_period = resolve_servo_period(LaunchConfiguration("servo_period").perform(context), rate)
    config["controller_manager"]["ros__parameters"]["update_rate"] = rate
    # Rewrite the exact node-specific key; wildcard overrides lose to this YAML.
    with tempfile.NamedTemporaryFile(mode="w", prefix="aubo_controllers_", suffix=".yaml", delete=False) as stream:
        yaml.safe_dump(config, stream)
        robot_controllers = stream.name

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
            "servo_mode:=",
            servo_mode,
            " control_period:=", str(1.0 / rate),
            " servo_period:=", str(servo_period),
            " ",
            "aubo_type:=",
            aubo_type,
            " ",
           
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [get_package_share_directory(moveit_config_package.perform(context)), "config", "moveit.rviz"]
    )

    # Humble upstream main ships the driver-owned control node. setcap and
    # aubo_ros2_control_node.cpp target this executable, not ros2_control_node.
    control_node = Node(
        package="aubo_ros2_driver",
        executable="aubo_ros2_control_node",
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
        condition=IfCondition(effective_launch_rviz),
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

    io_and_status_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        condition=UnlessCondition(use_fake_hardware),
        arguments=[
            "io_and_status_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    freedrive_mode_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        condition=UnlessCondition(use_fake_hardware),
        arguments=[
            "freedrive_mode_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    controller_stopper_node = Node(
        package="aubo_ros2_driver",
        executable="controller_stopper",
        condition=UnlessCondition(use_fake_hardware),
        output="both",
    )

    dashboard_client_node = Node(
        package="aubo_ros2_driver",
        executable="dashboard_client",
        name="dashboard_client",
        condition=UnlessCondition(use_fake_hardware),
        output="both",
        parameters=[
            {
                "robot_ip": robot_ip,
                "port": dashboard_port,
                "robot": dashboard_robot,
            }
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

    # Load/configure auxiliary plugins before starting motion. Loading a plugin
    # while Servo is live can block controller_manager's controller-list lock.
    chain = [joint_state_broadcaster_spawner]
    if use_fake_hardware.perform(context).lower() != "true":
        chain += [io_and_status_controller_spawner, freedrive_mode_controller_spawner]
    chain += [initial_joint_controller_spawner]
    def continue_after_success(next_action):
        def on_exit(event, context):
            if event.returncode != 0:
                return [LogInfo(msg="Controller spawner failed; motion startup aborted."),
                        EmitEvent(event=Shutdown(reason="controller startup failed"))]
            return [next_action]
        return on_exit
    spawner_chain = [RegisterEventHandler(OnProcessExit(
        target_action=previous, on_exit=continue_after_success(next_action)))
        for previous, next_action in zip(chain, chain[1:])]
    delayed_controller_spawners = RegisterEventHandler(OnProcessStart(
        target_action=control_node,
        on_start=[TimerAction(period=2.0, actions=[chain[0]])]))

    return [
        control_node,
        robot_state_publisher_node,
        rviz_node,
        controller_stopper_node,
        dashboard_client_node,
        *spawner_chain,
        delayed_controller_spawners,
    ]


def generate_launch_description():
    declared_arguments = [DeclareLaunchArgument(
        "update_rate", default_value="",
        description="Explicit cyclic rate in Hz; overrides the controller profile."),
        DeclareLaunchArgument("servo_period", default_value="0.01",
            description="servoJoint t in seconds, or auto=1/update_rate; default .01 preserves existing profiles."),
        DeclareLaunchArgument("controller_profile", default_value="auto",
            description="auto: ES models 100Hz, others YAML; es:100Hz; legacy:200Hz; custom:YAML.")]



    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="aubo_ros2_driver",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("use_calibrated", default_value="false",
                              description="Load the calibrated URDF; fail if missing.")
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
            default_value="127.0.0.1",
            description="IP of robot/controller. ARCS sim-as-real on this PC must be 127.0.0.1 (not LAN IP). Used only if use_fake_hardware is false."
            "Used only if 'use_fake_hardware' parameter is false.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "servo_mode",
            default_value="1",
            description="Servo mode select value passed to MotionControl.setServoModeSelect().",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "dashboard_port",
            default_value="9012",
            description="Dashboard / WebSocket JSON-RPC port for dashboard_client.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "dashboard_robot",
            default_value="rob1",
            description="Robot name prefix for dashboard_client JSON-RPC.",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "remote_viz",
            default_value="false",
            description="true=不启本机 RViz（多机通讯）；PC 另起 "
            "aubo_moveit_config remote_moveit_rviz.launch.py",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
