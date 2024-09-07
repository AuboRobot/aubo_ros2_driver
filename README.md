# AUBO_ROS2 #
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

ROS2 stack for **AUBO collaborative robots**. This package contains launch and configuration setups to quickly get started using the driver.

## Contributing

We strongly encourage you to help improve this documentation. Please consider helping improve the package, port other aubo robots from [aubo_description](https://github.com/AuboRobot/aubo_description/tree/main/urdf), and write new tutorials. We recommend you read the quality standards below as well as the [How to Write a MoveIt Tutorial](https://moveit.picknik.ai/main/doc/how_to_contribute/how_to_write_tutorials.html) page.

If you find an issue with the package you are not able to fix yourself, please [open an issue on Git](https://github.com/AuboRobot/aubo_ros2_driver/issues/new) or open a PR with proposed changes.

## Features ##
- integration with `ros2_control`
- robot drivers for ARCS for position control
- dedicated sensors and broadcasters to get data from the robot
- dedicated controllers, add gpio control(io、tool、modbus)
- integration with Moveit2 (OMPL, PILZ and servo)

## Available Packages in this Repository ##
- `aubo_bringup` - launch and run-time configurations
- `aubo_controllers` - implementation of dedicated controllers
- `aubo_description` - robot description and configuration files
- `aubo_hardware` - hardware interfaces for communication with the robot
- `aubo_msgs` - all interface definitions for aubo_ros2_driver

## Getting Started
***Required setup : Ubuntu 22.04 LTS***

1.  Install `ros2` packages. The current development is based of `ros2 humble`. Installation steps are described [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
2. Source your `ros2` environment:
    ```shell
    source /opt/ros/humble/setup.bash
    ```
    **NOTE**: The ros2 environment needs to be sources in every used terminal. If only one distribution of ros2 is used, it can be added to the `~/.bashrc` file.
3. Create a new ros2 workspace:
    ```shell
    mkdir -p ~/aubo_ros2_ws/src
    ```
4. Pull relevant packages, install dependencies, compile, and source the workspace by using:
    ```shell
    cd ~/aubo_ros2_ws
    git clone https://github.com/AuboRobot/aubo_ros2_driver.git src/aubo_ros2_driver
    vcs import src < src/aubo_ros2_driver/aubo_ros2.repos
    rosdep install --ignore-src --from-paths . -y -r
    sudo apt remove ros-humble-moveit*
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
    source install/setup.bash
    ```
**NOTE:** The `aubo_ros2.repos` file contains moveit2 src links to ros2 packages that need to be source-built to use their newest features([MoveItPy](https://github.com/moveit/moveit2/issues/2663)).If build quit,please [config system zram >=8Gb](https://github.com/moveit/moveit2/issues/2083) and build limited to 1 by selecting the sequential executor with `--executor sequential`.

5. build select packages by using`--packages-select`:
   ```shell
   colcon build --symlink-install --packages-select aubo_bringup aubo_controllers aubo_description aubo_hardware aubo_moveit2 aubo_msgs
   ```

## Usage

:warning: **SAFETY FIRST**:warning:
*An industrial robot is not a toy and you may harm yourself due to misuse. In general it is best practice to test your code at first in simulation and then in low speed mode. Before using the robot, make yourself familiar with the safety instructions provided by the AUBO manuals.*

### On the Robot side:

After the robot, controller, and teach pendant are installed and connected to the power supply, quickly start the system as described below:

Press the power switch of the controller and wait until the power indicator light and standby indicator light go on.

See the Hardware Manual for the power-on procedure of the controller.
Press the power switch on the teach pendant to start the AUBO SCOPE software.

In the [Robot Connection] window, enter the robot’s IP address and port number, and tap [Connect]. After the robot is successfully connected, the [Robot Connection] window quits automatically.

Select and determine the required tools.

Please stand outside the reach (work radius) of the robot.
Tap [Power on] on the touch screen, select payload, and tap [Continue]. The robot will be activated. After the robot is activated, the page will change, and the [Startup] button will be displayed.

Tap [Startup] to release the brake system. When the robot vibrates and clicks, the robot system is started and ready for operation.
**HINT:** In the proposed default setup of this package, the robot and the control PC are communicating through ARCS on the `AUBO Option Network Interface` with the following setup:
- Robot : `IP = X.X.X.X`

For further instructions concerning the installation and setup of ARCS, please refer to [AUBO documentation](https://arcs.aubo-robotics.cn/).

### On ROS2 side:
The `aubo_bringup` package contains 2 main launch files: one examples and the main driver launcher
- `joy_servo_teleop.launch.py` - launches a fake robot controlled by a joystick using `moveit_servo`
- `aubo.launch.py` - is the main launcher giving access to all feaures of the driver.

The arguments for launch files can be listed using
```shell
ros2 launch aubo_bringup <launch_file_name>.launch.py --show-args
```
The most relevant arguments of `aubo.launch.py` are:
- `robot_type` (default: "aubo_ES3") - AUBO Robot Type.
- `runtime_config_package` (default: "aubo_description") - name of the package with the controller's configuration in `config` folder. Usually the argument is not set, it enables use of a custom setup.
- `controllers_file` (default: "aubo_controllers.yaml"- YAML file with the controllers configuration.
- `description_package` (default: "aubo_description") - Description package with robot URDF/xacro files. Usually the argument is not set, it enables use of a custom description.
- `description_file` (default: "aubo.config.xacro") - URDF/XACRO description file with the robot.
- `tf_prefix` (default: "") - Prefix of the joint names, useful for multi-robot setup. If changed than also joint names in the controllers' configuration have to be updated. Expected format `<tf_prefix>/`.
- `namespace` (default: "/") - Namespace of launched nodes, useful for multi-robot setup. If changed than also the namespace in the controllers configuration needs to be updated. Expected format `<ns>/`.
- `use_sim` (default: "false") - Start robot in Gazebo simulation.
- `use_fake_hardware` (default: "true") - Start robot with fake hardware mirroring command to its states.
- `use_planning` (default: "false") - Start robot with Moveit2 `move_group` planning configuration for Pilz and OMPL.
- `use_servoing` (default: "false") - Start robot with Moveit2 servoing.
- `robot_controller` (default: "joint_trajectory_controller") - Robot controller to start.
- `launch_rviz` (default: "true") - Start RViz2 automatically with this launch file.
- `robot_ip` (default: "192.170.10.2") - Robot IP of ARCS interface.
- `initial_positions_file` (default: "initial_positions.yaml") - Configuration file of robot initial positions for simulation.
- `base_frame_file` (default: "base_frame.yaml") - Configuration file of robot base frame wrt the World frame.

As an example, to run the `aubo_ES3` on the real hardware with real ip(eg 192.168.3.120), run

```shell
ros2 launch aubo_bringup aubo.launch.py use_fake_hardware:="false" aubo_type:="aubo_ES3" robot_ip:=192.168.3.120 use_planning:="true"
```
or
```shell
ros2 launch aubo_bringup aubo.launch.py use_fake_hardware:=False aubo_type:=aubo_ES3 robot_ip:=192.168.3.120 use_planning:=True
```
**HINT**: list all loaded controllers using `ros2 control list_controllers` command.


**NOTE**: The package can simulate hardware with the ros2_control `FakeSystem`. This is the default behavior. This emulator enables an environment for testing of "piping" of hardware and controllers, as well as testing robot's descriptions. For more details see ros2_control documentation for more details.

### Example commands for setup testing
1. Start the simulated hardware, in a sourced terminal run
    ```shell
    ros2 launch aubo_bringup aubo.launch.py
    ```
    add the parameter `use_fake_hardware:="false"` to control the real robot.
2. Send joint trajectory goals to the hardware by using a demo node from [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) package by running
    ```shell
    ros2 launch aubo_hardware aubo_test_joint_trajectory_controller.launch.py
    ```

After a few seconds the robot should move.

## Practical information
### Domain setup
As by default ROS2 streams all data on the network, in order to avoid message interference, it is preferred to isolate the communications by defining domains per project/application.

To do so run `export ROS_DOMAIN_ID= [your_domain_id]`, with `[your_domain_id]` between 0 and 255.

