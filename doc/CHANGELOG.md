# 变更记录

## 未发布（2026-11-13）

### 修复

- 进入拖动示教改为和双臂 TEACH 一样：先退出伺服，再调用 `RobotManage.freedrive(true)`，并轮询 `isFreedriveEnabled()` 或 `isHandguideEnabled()`。柜端没有确认前，不再把本地拖动状态写成已开启，也不会停掉后续伺服输出。退出时调用 `freedrive(false)`，同样等到标志清除。
- 2026-11-13 真机复测：伺服还在时 `freedrive(true)` 返回 0，但 250 ms 内标志仍为 false，驱动随即重新进入伺服。先停掉 `joint_trajectory_controller` 后再调用，约 0.2 s 后 `isFreedriveEnabled` 与 `isHandguideEnabled` 都为 true，`getRobotControlMode` 为 `Freedrive`。确认等待改为伺服退出后再等 100 ms，之后最多 2 s。服务等待改为 8 s。

- 机械臂未上电时启动驱动，`aubo_ros2_control_node` 不再退出。原因：`on_activate` 在 2 秒内等不到新鲜且静止的 RTDE 反馈就返回 ERROR，controller_manager 中止，`joint_state_broadcaster` 的 spawner 120 秒后超时，launch 随之关闭 `dashboard_client` 和 MoveIt。现在真机模式下用 `hardware_components_initial_state.inactive` 让硬件先停在 inactive。`joint_state_broadcaster` 照常激活；`io_and_status_controller`、`freedrive_mode_controller`、`joint_trajectory_controller` 以 `--inactive` 加载。`/robot_manage/*` 与 `/jsonrpc_service` 在机械臂断电时可用。
- 新增 `on_configure`：只建立 RPC/RTDE 连接和反馈订阅，不上电、不进伺服。硬件 inactive 时 `/joint_states` 也是真实关节角，RViz/MoveIt 不再显示零位。`OnActive` 只在尚未订阅时订阅，断开连接时清掉订阅标记。
- `controller_stopper` 新增参数 `auto_activate_hardware`（默认 false）、`hardware_component`（默认 `aubo`）、`hardware_poll_period`（默认 1.0 s，下限 0.2 s）、`activate_controllers`（默认空）。`aubo_control.launch.py` 对真机打开这项功能，并把 `io_and_status_controller`、`freedrive_mode_controller` 和初始轨迹控制器填入 `activate_controllers`。开启后，每次轮询经 `/jsonrpc_service` 读 `RobotState.getRobotModeType`；到 Running 时，经 `/controller_manager/set_hardware_component_state` 把硬件切到 active。然后激活上次硬件 active 时处于 active 状态的控制器；硬件从未 active 过时，激活 `activate_controllers`。硬件激活失败时，下次轮询重试。
- 修复机械臂已是 Running 时启动，控制器留在 inactive。硬件在 spawner 加载控制器之前就被激活，当时没有可激活的控制器，之后加载的控制器也没人去激活。现在硬件激活后进入待恢复状态，每次轮询都会激活新加载的目标控制器，直到目标全部 active。待恢复期间不更新控制器快照。
- `robot_description.hardware_name()` 从 URDF 读取唯一的 `ros2_control` 系统名，launch 不再写死硬件名。新增依赖 `lifecycle_msgs`。

### 验证

- `colcon build --packages-select aubo_ros2_driver --symlink-install` 通过。`colcon test`（不含 lint）共 25 项，0 失败；`test_robot_description.py` 共 6 项全部通过。
- 2026-11-13 真机 172.16.31.105：
  - 机械臂处于 PowerOff 时冷启动全栈，节点均未退出。硬件为 inactive，`joint_state_broadcaster` 为 active，`/joint_states` 约 110 Hz，内容为实际关节角，其余 3 个控制器为 inactive。
  - 依次上电、等到 Idle、`startup` 后，`controller_stopper` 打印 `Robot is Running; hardware 'aubo' activated.` 并激活其余 3 个控制器。
  - 硬件 active 时软急停（下电），驱动不退出，也没有锁存故障。
  - 机械臂已是 Running 时重启全栈：硬件约 1 秒后激活。随后 io 控制器在加载后约 0.15 秒被激活，freedrive 和轨迹控制器加载完成后在下一次轮询被激活，最终 4 个控制器都是 active。

### 验收范围

- 控制柜网络不可达时，`on_init`/`on_configure` 仍会失败，节点仍会退出，这次没有处理。`on_deactivate` 仍断开连接，手动 deactivate 后 inactive 期间没有反馈。
- 下电瞬间约 0.6 秒内会连续出现 `setServoModeSelect returned -3` 日志，这是改动前就有的行为，这次没有修改。
## 未发布（2026-11-02）

### 修复

- 通用 `controller_profile:=auto` 对所有型号沿用 YAML，不再按型号名前缀选择频率；ES 的 100 Hz 通过显式 `es` profile 选择，显式 `update_rate` 仍优先。
- 非 Running/Normal（或 ReducedMode）状态及 SDK 207～212 返回码暂停输出，不立即锁存；RPC 安全状态拒绝后必须收到更新的就绪反馈才能恢复。恢复时同步当前位置并清零速度命令。
- 短时反馈超时、会话退出和 RPC 异常先暂停恢复；反馈连续缺失至少 1 秒、会话重建/RPC 连续失败达到 1 秒或 Servo 错误预算耗尽仍锁存故障。
- 未实现的 `speedServo()` 保持非致命处理，仅提示不支持，不发送速度命令；文档明确其支持限制。
- 默认轨迹控制器改为 `aubo_controllers/AuboJointTrajectoryController`，通过 `motion_status/execution_state` 将安全暂停或硬件故障传给 action，返回 `ABORTED` 和非零错误码。硬件错误状态在 SDK 清理后保留，恢复后须提交新目标，旧轨迹不续跑。
- 六轴目标位置容差设为 0.01 rad，停止速度容差为 0.02 rad/s，到位等待上限为 0.5 秒，避免反馈冻结时仅按时间结束判为成功。
- 保持 `aubo_description` 指向公开提交 `47fa5e0`；驱动侧配置生成描述的硬件插件与 Servo 参数，兼容该模型版本并恢复纯 mock 启动。
- 增加可重复执行的模型准备脚本，仅移除 6 个未被模型引用的悬空 DAE 链接，修复 Humble `--symlink-install`；不替换 mesh，不依赖未公开子模块提交。

### 验证

- 当前工作区和独立 clone 的 `--symlink-install` 构建均通过，8 个包完成；最后一轮控制器及驱动增量构建通过。
- 控制器和驱动测试汇总 33 项，0 错误、0 失败。覆盖安全返回码、新鲜反馈门控、错误清理后状态保留、action 中止及冻结反馈不能到位等行为。
- 完整 mock 链路通过 ros2_control GPIO 注入暂停/故障，确认 action 返回 `ABORTED`，恢复后新目标返回 `SUCCEEDED`；全程不连接 SDK。
- 2026-11-02 ES3 真机确认：碰撞中断后保持 ROS 驱动运行，操作者手动上电、启动并恢复就绪状态后，驱动自动重新进入 Servo 模式 1，无需 switch mode 或重启；重新提交的新 action goal 成功执行。README 与 ES 现场记录补充恢复步骤及 ROS 控制机制。

### 验收范围

- ES3 本次 ROS 层碰撞恢复与新目标执行已完成真机验证；其他型号/固件及实际跟踪容差仍需各自验收。驱动不会自动解除保护停、清除柜端故障、上电或松刹车。
- 本次仅处理 ROS 层。柜端停止时间超限、锁刹车超时与断电原因不在此次验收范围，不视为本次 ROS 修复已解决。
