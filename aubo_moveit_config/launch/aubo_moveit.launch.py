import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


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

    description_package = LaunchConfiguration("description_package").perform(context)
    moveit_config_package = LaunchConfiguration("moveit_config_package").perform(
        context
    )
    description_file = LaunchConfiguration("description_file").perform(context)
    aubo_type = LaunchConfiguration("aubo_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    moveit_config_file = LaunchConfiguration("moveit_config_file").perform(context)
    if not moveit_config_file:
        moveit_config_file = "config/aubo_robot.srdf"

    description_share = get_package_share_directory(description_package)
    moveit_share = get_package_share_directory(moveit_config_package)
    _ensure_description_xacro(description_share, aubo_type.perform(context))

    moveit_config = (
        MoveItConfigsBuilder("aubo", package_name=moveit_config_package)
        .robot_description(
            file_path=os.path.join(
                description_share, "urdf/xacro/inc", description_file
            ),
            mappings={
                "aubo_type": aubo_type,
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(moveit_share, moveit_config_file)
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

    moveit_controllers_path = os.path.join(
        moveit_share, "config", "moveit_controllers.yaml"
    )
    with open(moveit_controllers_path, "r", encoding="utf-8") as controllers_file:
        moveit_simple_controller_manager = yaml.safe_load(controllers_file)

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
                    "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
                    "wait_for_initial_state_timeout": 10.0,
                },
            },
        ],
    )

    rviz_config = os.path.join(moveit_share, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
        ],
    )

    return [move_group_node, rviz_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "description_package",
                default_value="aubo_description",
                description="Package that provides the robot xacro.",
            ),
            DeclareLaunchArgument(
                "moveit_config_package",
                default_value="aubo_moveit_config",
                description="Package that provides MoveIt config files.",
            ),
            DeclareLaunchArgument(
                "description_file",
                default_value="aubo_ros2.xacro",
                description="Robot description xacro file name.",
            ),
            DeclareLaunchArgument(
                "aubo_type",
                default_value="aubo_i5",
                description="Robot model to pass into the description xacro.",
            ),
            DeclareLaunchArgument(
                "moveit_config_file",
                default_value="",
                description="Optional SRDF path relative to the MoveIt config package. Empty uses the generic aubo_robot.srdf.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch RViz together with move_group.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
