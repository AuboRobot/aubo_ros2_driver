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

### 伺服模式参数

驱动在运动控制器下发 `position` 关节目标时进入 SDK 伺服运动模式，用于持续执行 `servoJoint` 控制；运动控制器停止后会退出伺服模式。本驱动使用 `MotionControl.setServoModeSelect()` / `getServoModeSelect()` 进入和确认伺服模式，不再使用旧的 `setServoMode(true/false)` 接口。

`aubo_control.launch.py` 支持通过 `servo_mode` 选择进入模式，默认值为 `1`。通常无需显式传入；如果现场需要切换到其它伺服模式，可以在启动驱动时追加参数。

`common_interface` 中定义的取值如下：

| `servo_mode` | 含义 |
| --- | --- |
| `0` | 退出伺服模式，驱动停止伺服时固定使用该值 |
| `1` | 截断式规划伺服模式，默认进入模式 |
| `2` | 透传模式，直接下发 |
| `3` | 透传模式，缓存下发 |
| `4` | 1ms 透传模式，缓存下发；需控制器支持 1000Hz 点位消费，否则通常按 200Hz 使用 |
| `5` | 规划伺服模式 |
| `6` | 截断式规划伺服模式，可以叠加 SDK `ForceControl` 力控 |
| `7` | 规划伺服模式，可以叠加 SDK `ForceControl` 力控 |

模式推荐：

| 场景 | 推荐 |
| --- | --- |
| 非实时场景，如 MoveIt、JointTrajectoryController、普通 Linux 控制链路 | 默认使用 `1`；需要保证经过所有点位时可选 `5` |
| 实时场景，如外部控制器稳定周期下发点位 | 可选 `2` / `3`；控制器支持 1000Hz 点位消费时再考虑 `4` |
| 需要叠加力控 | 按场景选择 `6` 或 `7`；力控指 AUBO SDK `ForceControl` 类提供的机器人控制器层力控制 |

说明：

- 模式 `1` 会持续修正目标点和规划路线，更适合非实时控制链路；模式 `5` 会保证经过所有目标点。
- 直接使用 SDK `servoJoint` 发送点位流时，建议保持 `t` 与下发周期一致，且不小于机器人内部控制周期。
- 点位过密应重采样或合并冗余点，点位过远应重新定时或放慢轨迹。
- 带缓存的模式队列满时会返回错误；实时跟踪可丢弃过期点并发送最新点，离线轨迹应等待并重试当前点。

```bash
ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5 robot_ip:=192.168.127.128 \
  use_fake_hardware:=false servo_mode:=2
```

不同控制器和 SDK 版本对模式支持可能存在差异，实际可用取值请以当前控制器配套 SDK 文档为准。

当前驱动运动命令链路支持 `position` command interface。`velocity` command interface 已保留占位链路，激活后会正常消费速度命令并进入 `speedServo()`；只是 `speedServo()` 底层执行暂未实现，驱动会打印 ERROR，且不会向机器人下发速度伺服命令。

### IO、配置与拖动示教

真实硬件启动时，`aubo_control.launch.py` 会自动启动 `aubo_controllers` 包中的 `io_and_status_controller`、`freedrive_mode_controller`，以及驱动包中的 `controller_stopper` 和 `dashboard_client`。其中 `io_and_status_controller` 从 ros2_control state interfaces 读取状态并发布 ROS 话题，发布数组长度由 `AuboHardwareInterface` 实际导出的接口数量决定；IO 写服务使用固定 command payload 接口，默认启用，`pin` 参数直接透传给 SDK。`freedrive_mode_controller` 通过 command interfaces 写入拖动示教命令：

- `/io_control/state`：发布 `aubo_msgs/msg/IoControlState`，包含标准 IO、可配置 IO、末端 IO、末端电压和末端配置状态。
- `/robot_manage/state`：发布 `aubo_msgs/msg/RobotManageState`，包含拖动示教状态、free axes 和 feature。
- `/robot_config/state`：发布 `aubo_msgs/msg/RobotConfigState`，包含 TCP offset、payload 质量和质心。
- `/io_control/*`：提供 IO 写入和末端 IO 配置服务，默认启用；无效 `pin` 会由 SDK 返回错误码，驱动在日志中打印。
- `/robot_config/*`：提供 payload、TCP offset 等配置服务，默认启用。
- `/robot_manage/set_handguide`：提供拖动示教服务。
- `/robot_manage/poweron`、`/robot_manage/startup`、`/robot_manage/poweroff` 等：由 `dashboard_client` 通过 9012 WebSocket JSON-RPC 提供机器人生命周期服务。
- `/freedrive_mode_controller/enable_freedrive_mode`：提供拖动示教 Bool 话题入口。

`/io_control/state`、`/robot_manage/state` 和 `/robot_config/state` 跟随 `controller_manager.update_rate` 发布，不再额外设置 IO 状态发布限速。

实时控制、RTDE 状态、IO、Payload、TCP offset 与拖动示教仍收敛在 `AuboHardwareInterface` 和 ros2_control controllers。机器人上电、启动、断电、松刹车、解除保护停等生命周期能力由 `dashboard_client` 通过 9012 WebSocket JSON-RPC 提供；这些操作可能改变机器人运行状态，驱动会在机器人不可运动时暂停运动输出并失效本地伺服模式状态，恢复到可运动状态后会重新进入所选 `servo_mode`。`aubo_controllers` 不连接 SDK，也不订阅 RTDE。`controller_stopper` 只根据 `/robot_manage/state` 编排运动控制器启停。

查看当前 IO 状态：

```bash
ros2 topic echo /io_control/state
ros2 topic echo /robot_manage/state
ros2 topic echo /robot_config/state
```

常用写操作示例：

```bash
# 标准数字输出 0 置 ON；output_type: 0=standard, 1=configurable, 2=tool
ros2 service call /io_control/set_digital_output aubo_msgs/srv/SetDigitalOutput \
"{output_type: 0, pin: 0, state: true}"

# 标准模拟输出 0；output_type: 0=standard, 1=tool
ros2 service call /io_control/set_analog_output aubo_msgs/srv/SetAnalogOutput \
"{output_type: 0, pin: 0, state: 0.5}"

# 配置末端电压，常用取值为 0、12、24
ros2 service call /io_control/set_tool_voltage aubo_msgs/srv/SetToolVoltage \
"{domain: 24}"

# 配置末端 IO 方向，input=true 表示输入，input=false 表示输出
ros2 service call /io_control/set_tool_io_input aubo_msgs/srv/SetToolIoInput \
"{pin: 0, input: false}"
```

末端 IO 既可以读写当前信号，也可以配置工作方式。建议按以下顺序配置：

1. 先订阅 `/io_control/state`，确认当前机器人实际返回的 IO 数量和末端能力。
2. 如果末端设备需要供电，先调用 `/io_control/set_tool_voltage` 配置末端电源电压，再通过 `/io_control/state` 确认 `tool_voltage_output_domain`。
3. 对末端数字 IO，先调用 `/io_control/set_tool_io_input` 配置方向。
4. 如果该口作为输入使用，再按需要调用 `/io_control/set_tool_io_config` 配置 `DIGITAL_INPUT_ACTION`。
5. 如果该口作为输出使用，再按需要调用 `/io_control/set_tool_io_config` 配置 `DIGITAL_OUTPUT_RUNSTATE`；需要手动写输出值时，runstate 应配置为 SDK 中的 `None`。之后调用 `/io_control/set_digital_output` 写末端输出。
6. 对末端模拟输入，调用 `/io_control/set_tool_io_config` 配置 `ANALOG_INPUT_DOMAIN`，再读取 `/io_control/state` 中的 `tool_analog_inputs`。
7. 对末端模拟输出，调用 `/io_control/set_tool_io_config` 配置 `ANALOG_OUTPUT_DOMAIN` 和 `ANALOG_OUTPUT_RUNSTATE`，之后调用 `/io_control/set_analog_output` 写末端模拟输出。

`/io_control/set_tool_io_config` 的 `config_type` 取值：

| `config_type` | 含义 |
| --- | --- |
| `0` | `DIGITAL_INPUT_ACTION` |
| `1` | `DIGITAL_OUTPUT_RUNSTATE` |
| `2` | `ANALOG_INPUT_DOMAIN` |
| `3` | `ANALOG_OUTPUT_DOMAIN` |
| `4` | `ANALOG_OUTPUT_RUNSTATE` |

`value` 对应 SDK 的枚举值或 domain 值，请以当前控制器配套 SDK 文档为准。

拖动示教由 `freedrive_mode_controller` 控制。需要指定 free axes 或 feature 时，使用 `/robot_manage/set_handguide` 服务。`free_axes` 必须是 5 个 `0/1` 值；SDK 当前文档只将其描述为拖动方向开关，没有公开每个 index 的具体方向命名。`feature` 必须是 6 个值，留空时沿用驱动当前读取到的值。

```bash
# 进入拖动示教；free_axes 或 feature 留空时使用当前值
ros2 service call /robot_manage/set_handguide aubo_msgs/srv/SetHandguide \
"{enable: true, free_axes: [1, 1, 1, 1, 1], feature: [0, 0, 0, 0, 0, 0]}"

# 退出拖动示教
ros2 service call /robot_manage/set_handguide aubo_msgs/srv/SetHandguide \
"{enable: false, free_axes: [], feature: []}"
```

只需要按默认参数进入或退出时，也可以发布 Bool 话题：

```bash
ros2 topic pub --once /freedrive_mode_controller/enable_freedrive_mode \
std_msgs/msg/Bool "{data: true}"

ros2 topic pub --once /freedrive_mode_controller/enable_freedrive_mode \
std_msgs/msg/Bool "{data: false}"
```

进入拖动示教时，`AuboHardwareInterface` 会退出伺服模式，并在拖动示教期间暂停 `servoJoint` / `speedServo` 输出，避免运动控制器重新拉起伺服模式。同时，`controller_stopper` 会在 `/robot_manage/state` 进入拖动示教状态后停用当前 active 的 `position` / `velocity` 运动控制器，并在退出拖动示教后重新激活这些控制器。退出后可以通过 `ros2 control list_controllers -c /controller_manager` 确认 `joint_trajectory_controller` 已恢复 active。

Payload 和 TCP offset 配置：

```bash
ros2 service call /robot_config/set_payload aubo_msgs/srv/SetPayload \
"{mass: 1.2, center_of_gravity: {x: 0.0, y: 0.0, z: 0.05}}"

ros2 service call /robot_config/set_tcp_offset aubo_msgs/srv/SetTcpOffset \
"{tcp_offset: [0, 0, 0.1, 0, 0, 0]}"
```

ros2_control 的 `io_control/get_*`、`io_control/set_*`、`robot_config/*` 和 `robot_manage/*` interface 仍会导出，主要用于控制器集成和底层调试。读状态接口按实际硬件数量导出；写命令接口是固定 payload 语义，例如数字输出使用 `set_digital_output_type`、`set_digital_output_pin`、`set_digital_output_value` 和 `set_digital_output_trigger`，不按 pin 数量导出 command interfaces。

机器人生命周期服务：

```bash
# 上电
ros2 service call /robot_manage/poweron std_srvs/srv/Trigger {}

# 启动
ros2 service call /robot_manage/startup std_srvs/srv/Trigger {}

# 断电
ros2 service call /robot_manage/poweroff std_srvs/srv/Trigger {}

# 松开/锁定刹车
ros2 service call /robot_manage/release_robot_brake std_srvs/srv/Trigger {}
ros2 service call /robot_manage/lock_robot_brake std_srvs/srv/Trigger {}

# 解除保护停
ros2 service call /robot_manage/unlock_protective_stop std_srvs/srv/Trigger {}
```

## 驱动真实机械臂 aubo_i5 单点轨迹执行 demo（修改机器人对应 `robot_ip`、`aubo_type`）

```bash
source install/setup.bash
ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5 robot_ip:=192.168.127.128 \
  use_fake_hardware:=false
ros2 launch ros_joints_plan joints_plan.launch.py aubo_type:=aubo_i5
```

## Dashboard Client 与 JSON-RPC 调试服务（修改机器人对应 `robot_ip`）

`dashboard_client` 默认连接机器人 WebSocket 端口 `9012`，提供 `/robot_manage/*`
生命周期服务，同时保留 `/jsonrpc_service` 作为调试和高级排查的 JSON-RPC 透传入口。
常规 IO、拖动示教、Payload、TCP offset 等用户能力优先使用上面的 ROS topic/service；
这些能力会通过 ros2_control controller 与驱动协同，避免绕过运动控制链路。

```bash
source install/setup.bash
ros2 launch aubo_ros2_driver dashboard_client.launch.py robot_ip:=127.0.0.1 port:=9012 log_level:=info
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
