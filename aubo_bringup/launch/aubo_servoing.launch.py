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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
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

    # Initialize Arguments
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    tf_prefix = LaunchConfiguration('tf_prefix')
    namespace = LaunchConfiguration('namespace')
    aubo_type = LaunchConfiguration("aubo_type")

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
            'aubo_type:=',
            aubo_type,
            ' ',
            'description_package:=',
            description_package,
            ' ',
            'namespace:=',
            namespace,
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    # Get SRDF via xacro
    robot_description_semantic_content = ParameterValue(
    Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "srdf", "aubo.srdf.xacro"]
            ),
            " ",
            "name:=",
            "aubo",
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            'description_package:=',
            description_package,
        ]
    ),value_type=str)

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(description_package), "moveit2", "kinematics.yaml"]
    )

    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "aubo_arm"}

    # Get parameters for the Servo node
    servo_params = PathJoinSubstitution([
            FindPackageShare(description_package),
            'moveit2',
            'aubo_moveit2_servo_config.yaml',
        ]
    )

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name="servo_node",
        output='screen',
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            robot_description,
            robot_description_kinematics,
            robot_description_semantic
        ],
    )

    return LaunchDescription([servo_node, ])
