# aubo_ros2_driver

遨博机器人ROS2驱动

## ROS 2 Jazzy 适配状态

当前仓库已经完成 ROS 2 Jazzy 的初步适配，`aubo_ros2_driver`、`aubo_description`、`aubo_moveit_config` 可在 Jazzy 环境下完成基础启动、状态显示、MoveIt 规划与轨迹执行。

本次适配已处理的关键问题包括：

- MoveIt OMPL 规划配置适配 Jazzy 参数格式。
- MoveIt 控制器管理器参数补齐，支持 `joint_trajectory_controller` 执行。
- `joint_limits.yaml` 补充加速度限制，修复 `AddTimeOptimalParameterization` 失败问题。
- `SRDF` 虚拟关节修正为 `world -> base_link`。
- RViz `Planning Scene Topic` 修正为 `/monitored_planning_scene`，使 MotionPlanning 场景机器人状态可正常刷新。

说明：

- 当前文档中的 MoveIt 相关启动方式，均以 Jazzy 适配后的配置为准。
- 如果是实机调试，仍建议优先完成 URDF 校准后再做 MoveIt 验证。

## 在 rviz 中查看 aubo 机器人模型（以 aubo_i5 为例）

```bash
ros2 launch aubo_description aubo_viewer.launch.py
```

## 驱动真实机械臂前建议优先完成 URDF 校准

建议先根据机器人当前控制器返回的校准补偿生成校准版 URDF，再进行实机驱动、MoveIt 规划和轨迹验证。

如果直接使用未校准的默认 URDF，可能存在以下风险：

- 规划模型与真实机器人运动学参数不一致，末端位姿存在偏差。
- RViz、MoveIt 中显示的姿态和实机反馈不完全一致，影响问题定位。
- TCP 验证、轨迹复现、离线点位比对等依赖模型精度的功能，结果可能不可靠。

推荐方法：

```bash
cd <your_ros2_ws>
source /opt/ros/jazzy/setup.bash
python3 src/aubo_description/scripts/calibrate_urdf_dh.py \
  --robot-model aubo_i5 \
  --robot-ip 192.168.127.128
colcon build --packages-select aubo_description
source install/setup.bash
```

说明：

- 生成结果默认写入 `src/aubo_description/urdf/<robot_model>_calibrated.urdf`。
- `--robot-ip` 需要显式传入。
- 运行前请确认当前 Python 环境可以导入 `numpy` 和 `pyaubo_sdk`。
- 生成后需要单独重新编译 `aubo_description` 包。

## 驱动真实机械臂 aubo_i5（修改机器人对应 `robot_ip`、`aubo_type`）

终端 1：

```bash
cd <your_ros2_ws>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5 robot_ip:=192.168.127.128 \
  use_fake_hardware:=false
```

终端 2：

```bash
cd <your_ros2_ws>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=aubo_i5
```

在 ROS 2 Jazzy 下，`aubo_moveit.launch.py` 已完成基础适配，可直接用于 RViz + MoveIt 联调。`launch_rviz` 默认为 `true`。

## 驱动真实机械臂 aubo_i5 单点轨迹执行 demo（修改机器人对应 `robot_ip`、`aubo_type`）

终端 1：

```bash
cd <your_ros2_ws>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5 robot_ip:=192.168.127.128 \
  use_fake_hardware:=false
```

终端 2：

```bash
cd <your_ros2_ws>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ros_joints_plan joints_plan.launch.py aubo_type:=aubo_i5
```

在 ROS 2 Jazzy 下，`ros_joints_plan` 的 launch 配置也已同步到当前 MoveIt 适配方案。

## 服务节点驱动真实机械臂（修改机器人对应 `robot_ip`）

```bash
cd <your_ros2_ws>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch aubo_ros2_driver aubo_client.launch.py robot_ip:=127.0.0.1 log_level:=info
```

可选参数：

- `port`：TCP 服务端口，默认 `30004`
- `robot`：机器人前缀，默认 `rob1`
- `log_level`：日志级别，默认 `info`

## 调用示例

```bash
cd <your_ros2_ws>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call /jsonrpc_service aubo_msgs/srv/JsonRpc \
"{cls: 'RobotState', func: 'getTcpPose', params: '[]'}"
```

## 响应示例

```bash
requester: making request: aubo_msgs.srv.JsonRpc_Request(cls='RobotState', func='getTcpPose', params='[]')

response:
aubo_msgs.srv.JsonRpc_Response(result='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]', error='None')
```

## 异常响应示例（输入错误类 `RobotStat`）

```bash
requester: making request: aubo_msgs.srv.JsonRpc_Request(cls='RobotStat', func='getTcpPose', params='[]')

response:
aubo_msgs.srv.JsonRpc_Response(result='None', error='{"code": -32601, "message": "method not found: rob1.RobotStat.getTcpPose"}')
```

## aubo_sdk 接口参考文档

[aubo_sdk developer](https://docs.aubo-robotics.cn/arcs_api/index.html)
