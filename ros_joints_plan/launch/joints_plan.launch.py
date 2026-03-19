import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


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

    moveit_config_package = LaunchConfiguration("moveit_config_package").perform(context)
    moveit_config_file = LaunchConfiguration("moveit_config_file").perform(context)
    robot_xacro_file = LaunchConfiguration("robot_xacro_file").perform(context)
    aubo_type = LaunchConfiguration("aubo_type")
    support_package = LaunchConfiguration("support_package").perform(context)

    if not moveit_config_file:
        moveit_config_file = "aubo_robot.srdf"

    description_share = get_package_share_directory(support_package)
    moveit_share = get_package_share_directory(moveit_config_package)
    _ensure_description_xacro(description_share, aubo_type.perform(context))

    moveit_config = (
        MoveItConfigsBuilder("aubo", package_name=moveit_config_package)
        .robot_description(
            file_path=os.path.join(description_share, "urdf/xacro/inc", robot_xacro_file),
            mappings={"aubo_type": aubo_type},
        )
        .robot_description_semantic(
            file_path=os.path.join(moveit_share, "config", moveit_config_file)
        )
        .robot_description_kinematics(
            file_path=os.path.join(moveit_share, "config", "kinematics.yaml")
        )
        .joint_limits(
            file_path=os.path.join(moveit_share, "config", "joint_limits.yaml")
        )
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .trajectory_execution(
            file_path=os.path.join(moveit_share, "config", "moveit_controllers.yaml"),
            moveit_manage_controllers=False,
        )
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    moveit_controllers_path = os.path.join(moveit_share, "config", "moveit_controllers.yaml")
    with open(moveit_controllers_path, "r", encoding="utf-8") as controllers_file:
        moveit_simple_controller_manager = yaml.safe_load(controllers_file)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            os.path.join(description_share, "urdf/xacro/inc", robot_xacro_file),
            " ",
            "aubo_type:=",
            aubo_type,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
                "moveit_simple_controller_manager": moveit_simple_controller_manager,
                "planning_scene_monitor_options": {
                    "name": "planning_scene_monitor",
                    "robot_description": "robot_description",
                    "joint_state_topic": "/joint_states",
                    "attached_collision_object_topic": "/move_group/planning_scene_monitor",
                    "publish_planning_scene_topic": "/move_group/publish_planning_scene",
                    "monitored_planning_scene_topic": "/monitored_planning_scene",
                    "wait_for_initial_state_timeout": 10.0,
                },
            },
        ],
    )

    move_group_demo = Node(
        name="joints_plan",
        package="ros_joints_plan",
        executable="joints_plan",
        output="screen",
        parameters=[
            moveit_config.robot_description_semantic,
            moveit_config.robot_description,
            moveit_config.robot_description_kinematics,
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    return [move_group_node, move_group_demo, robot_state_pub_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_xacro_file",
                default_value="aubo_ros2.xacro",
                description="Xacro describing the robot.",
            ),
            DeclareLaunchArgument(
                "support_package",
                default_value="aubo_description",
                description="Name of the support package.",
            ),
            DeclareLaunchArgument(
                "moveit_config_file",
                default_value="aubo_robot.srdf",
                description="SRDF file name inside the MoveIt config package.",
            ),
            DeclareLaunchArgument(
                "moveit_config_package",
                default_value="aubo_moveit_config",
                description="MoveIt config package name.",
            ),
            DeclareLaunchArgument(
                "aubo_type",
                default_value="aubo_i5",
                description="Robot model to pass into the description xacro.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
