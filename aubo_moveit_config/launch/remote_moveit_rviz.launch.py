# -*- coding: utf-8 -*-
"""PC 端只启 MoveIt RViz（配合控制端 remote_viz:=true）。

控制端示例（ARCS sim-as-real，本机 loopback）：
  ros2 launch aubo_ros2_driver aubo_control.launch.py \\
    aubo_type:=aubo_ES3 robot_ip:=127.0.0.1 use_fake_hardware:=false \\
    remote_viz:=true launch_rviz:=false

PC / 另一台可视化机：
  ROS_DOMAIN_ID 与控制端一致后：
  ros2 launch aubo_moveit_config remote_moveit_rviz.launch.py \\
    aubo_type:=aubo_ES3 robot_ip:=127.0.0.1

不启 move_group / ros2_control；勿订阅高频图像话题。
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _ensure_description_xacro(description_share: str, aubo_type: str) -> None:
    urdf_dir = Path(description_share) / "urdf"
    target = urdf_dir / f"{aubo_type}.urdf.xacro"
    source = urdf_dir / f"{aubo_type}.urdf"
    if target.exists() and (not source.exists() or target.stat().st_mtime >= source.stat().st_mtime):
        return
    if not source.exists():
        # fall back to xacro/inc tree used by aubo_moveit
        return
    lines = source.read_text(encoding="utf-8").splitlines(keepends=True)
    if len(lines) >= 2 and "xmlns:xacro" not in lines[1]:
        lines[1] = lines[1].replace(">", ' xmlns:xacro="http://wiki.ros.org/xacro">', 1)
    lines = [line for line in lines if "<property" not in line]
    target.write_text("".join(lines), encoding="utf-8")


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    description_package = LaunchConfiguration("description_package").perform(context)
    moveit_config_package = LaunchConfiguration("moveit_config_package").perform(context)
    aubo_type = LaunchConfiguration("aubo_type").perform(context)
    use_calibrated = LaunchConfiguration("use_calibrated").perform(context).lower()
    if use_calibrated == "true":
        aubo_type += "_calibrated"
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"

    description_share = get_package_share_directory(description_package)
    moveit_share = get_package_share_directory(moveit_config_package)
    _ensure_description_xacro(description_share, aubo_type)

    description_file = LaunchConfiguration("description_file").perform(context)
    xacro_inc = os.path.join(description_share, "urdf/xacro/inc", description_file)
    typed_xacro = os.path.join(description_share, "urdf", f"{aubo_type}.urdf.xacro")
    if os.path.isfile(xacro_inc):
        file_path = xacro_inc
        mappings = {"aubo_type": aubo_type, "robot_ip": robot_ip, "use_fake_hardware": use_fake_hardware}
    else:
        file_path = typed_xacro
        mappings = {"robot_ip": robot_ip, "use_fake_hardware": use_fake_hardware}

    moveit_config = (
        MoveItConfigsBuilder("aubo", package_name=moveit_config_package)
        .robot_description(file_path=file_path, mappings=mappings)
        .robot_description_semantic(file_path=os.path.join(moveit_share, "config/aubo_robot.srdf"))
        .robot_description_kinematics(file_path=os.path.join(moveit_share, "config/kinematics.yaml"))
        .joint_limits(file_path=os.path.join(moveit_share, "config/joint_limits.yaml"))
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    rviz_config = os.path.join(moveit_share, "config", "moveit.rviz")
    return [
        LogInfo(
            msg="[remote_moveit_rviz] PC-side RViz only; expect TF/joint_states from control host. "
            f"robot_ip(placeholder)={robot_ip}"
        ),
        SetEnvironmentVariable(name="QT_QPA_PLATFORM", value="xcb"),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                moveit_config.planning_pipelines,
                {"use_sim_time": use_sim_time},
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("description_package", default_value="aubo_description"),
            DeclareLaunchArgument("moveit_config_package", default_value="aubo_moveit_config"),
            DeclareLaunchArgument("description_file", default_value="aubo_ros2.xacro"),
            DeclareLaunchArgument("aubo_type", default_value="aubo_ES3"),
            DeclareLaunchArgument(
                "robot_ip",
                default_value=os.environ.get("AUBO_ROBOT_IP", "127.0.0.1"),
                description="URDF 占位；ARCS sim-as-real 必须 127.0.0.1",
            ),
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument("use_calibrated", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
