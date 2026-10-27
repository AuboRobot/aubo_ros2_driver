# aubo_ros2_driver

遨博机器人ROS2驱动

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
cd /root/Desktop/aubo_ros2_ws
python3 src/aubo_description/scripts/calibrate_urdf_dh.py \
  --robot-model aubo_i5 \
  --robot-ip 192.168.127.128
colcon build --packages-select aubo_description
```

说明：

- 生成结果默认写入 `src/aubo_description/urdf/<robot_model>_calibrated.urdf`。
- `--robot-ip` 需要显式传入。
- 运行前请确认当前 Python 环境可以导入 `numpy` 和 `pyaubo_sdk`。
- 生成后需要单独重新编译 `aubo_description` 包。

## 驱动真实机械臂 aubo_i5（修改机器人对应 `robot_ip`、`aubo_type`）

```bash
source install/setup.bash
ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5 robot_ip:=192.168.127.128 \
  use_fake_hardware:=false
ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=aubo_i5
```

## 驱动真实机械臂 aubo_i5 单点轨迹执行 demo（修改机器人对应 `robot_ip`、`aubo_type`）

```bash
source install/setup.bash
ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5 robot_ip:=192.168.127.128 \
  use_fake_hardware:=false
ros2 launch ros_joints_plan joints_plan.launch.py aubo_type:=aubo_i5
```

## 服务节点驱动真实机械臂（修改机器人对应 `robot_ip`）

```bash
source install/setup.bash
ros2 launch aubo_ros2_driver aubo_client.launch.py robot_ip:=127.0.0.1 log_level:=info
```

## 独立 Topic 命令节点

该节点直接通过 AUBO SDK 控制机械臂，不使用 ros2_control 硬件插件。不要同时启动其他会控制同一机械臂的驱动。

```bash
source install/setup.bash
ros2 launch aubo_ros2_driver aubo_command.launch.py robot_ip:=192.168.127.128
```

命令 Topic：

- `/aubo/power_command` (`aubo_msgs/msg/AuboPowerCommand`)：`power_on=true` 上电并启动，`false` 下电。
- `/aubo/joint_move` (`aubo_msgs/msg/AuboJointMove`)：六轴关节目标，角度使用弧度，速度使用 rad/s，加速度使用 rad/s^2。
- `/aubo/linear_move` (`aubo_msgs/msg/AuboLinearMove`)：目标 `PoseStamped`，必须使用配置的 `base_frame`（默认 `base_link`）；位置单位米，速度/加速度单位 m/s、m/s^2。

### 上下电

上电命令会请求机械臂上电并启动（松刹车），等待进入 Running 模式：

```bash
ros2 topic pub --once /aubo/power_command aubo_msgs/msg/AuboPowerCommand \
"{command_id: 'power-on-001', power_on: true}"
```

下电命令：

```bash
ros2 topic pub --once /aubo/power_command aubo_msgs/msg/AuboPowerCommand \
"{command_id: 'power-off-001', power_on: false}"
```

### 关节运动

先确认机械臂处于可运动状态，并根据机器人型号、当前姿态和工作空间设置安全的六轴目标。以下仅为消息格式示例；位置单位为弧度：

```bash
ros2 topic pub --once /aubo/joint_move aubo_msgs/msg/AuboJointMove \
"{command_id: 'joint-001', positions: [0.0, -0.2, 0.3, 0.0, 0.2, 0.0], max_velocity: 0.2, max_acceleration: 0.4}"
```

### 直线运动

目标位姿以 `base_frame` 为参考坐标系，默认是 `base_link`。位置单位为米，方向使用四元数；速度单位为 m/s，加速度单位为 m/s^2，混合半径单位为米。下面的位姿只展示消息格式，使用前要确认目标在机械臂可达且无碰撞的工作空间内：

```bash
ros2 topic pub --once /aubo/linear_move aubo_msgs/msg/AuboLinearMove \
"{command_id: 'linear-001', target: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.30, y: 0.0, z: 0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, speed: 0.05, acceleration: 0.1, blend_radius: 0.0}"
```

### 查看执行结果

在另一个终端订阅状态 Topic：

```bash
ros2 topic echo /aubo/command_status
```

每条命令的 `command_id` 必须非空，建议每次请求使用唯一值。状态包括 `accepted`、`running`、`succeeded`、`failed` 和 `rejected`。节点一次只接收一条命令执行；正在执行时发布的新命令会以 `rejected` 状态返回。若收到 `failed` 或 `rejected`，请检查状态消息的 `message` 字段及节点日志。

该节点直接通过 AUBO SDK 控制机器人。不要同时启动其他会控制同一机械臂的驱动。上电/下电 Topic 不是急停接口；急停必须使用机器人控制柜或安全系统提供的硬件安全功能。

## 调用示例

```bash
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
