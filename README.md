# aubo_ros2_driver

遨博机器人ROS2驱动

## 安装与编译（Ubuntu 24.04 / ROS 2 Jazzy）

在工作区根目录执行；本机为 ARM64（aarch64）。模型由 Git 子模块提供，首次克隆后必须初始化：

```bash
cd ~/workspace
git -C src/aubo_ros2_driver submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
# 首次使用 rosdep 时按需执行 sudo rosdep init；已有配置无需重复初始化
rosdep update
rosdep install --from-paths src/aubo_ros2_driver --ignore-src -r -y --rosdistro jazzy \
  --skip-keys="gazebo_ros gazebo_ros2_control docker.io"
CMAKE_BUILD_PARALLEL_LEVEL=4 colcon build --base-paths src/aubo_ros2_driver --packages-skip aubo_gazebo \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`aubo_gazebo` 依赖 Gazebo Classic；上述 Jazzy 编译流程跳过此包及其两项依赖，不代表已验证 Gazebo 仿真。
`--base-paths` 避免把工作区内无关的 EtherCAT 源码纳入构建。
内存较少时可将 `CMAKE_BUILD_PARALLEL_LEVEL` 降为 `2`。Docker 仅为测试依赖，本次未安装；需要 Docker 测试时移除对应 skip key。
请以普通用户运行 rosdep，按提示使用 sudo 安装系统包；不要直接用 root 运行 rosdep，否则可能找不到用户的 rosdep 缓存。

### 本机依赖检查与排查记录（2026-10-29）

已执行实际 colcon 构建及 `rosdep check --from-paths src/aubo_ros2_driver --ignore-src`。
初始环境缺少以下系统依赖（安装操作需要 sudo）：

```bash
sudo apt update
sudo apt install \
  ros-jazzy-controller-manager ros-jazzy-controller-manager-msgs \
  ros-jazzy-hardware-interface ros-jazzy-controller-interface ros-jazzy-realtime-tools \
  ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller \
  ros-jazzy-position-controllers ros-jazzy-velocity-controllers \
  ros-jazzy-force-torque-sensor-broadcaster ros-jazzy-ros2-controllers-test-nodes \
  ros-jazzy-control-msgs ros-jazzy-moveit-msgs \
  ros-jazzy-moveit-ros-move-group ros-jazzy-moveit-kinematics ros-jazzy-moveit-planners \
  ros-jazzy-moveit-simple-controller-manager ros-jazzy-moveit-configs-utils \
  ros-jazzy-moveit-ros-visualization ros-jazzy-moveit-ros-warehouse \
  ros-jazzy-moveit-setup-assistant ros-jazzy-moveit-ros-planning-interface \
  ros-jazzy-warehouse-ros ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro nlohmann-json3-dev socat
```

以上为已检测到的缺项与规划接口依赖；完整安装以初始化子模块后的 rosdep 结果为准。
已按授权安装缺失依赖（apt 新增 101 个包），包括驱动运行依赖 `socat`。
`docker.io` 为驱动测试依赖，本次跳过。模型子模块已初始化，固定版本为 `47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1`。
模型包构建会在子模块 `urdf/` 下生成 `.urdf.xacro` 文件，因此 Git 可能显示该子模块含未跟踪文件。

常见问题：

- `Cannot locate rosdep definition for [aubo_description]`：先初始化子模块，确认目录内存在 `package.xml`。
- `tf` / `actionlib`：这是 ROS 1 依赖；`ros_joints_plan/package.xml` 已改为声明 ROS 2 的 tf2 和 MoveIt 规划接口依赖。
- 找不到 `controller_manager`、`hardware_interface` 等 CMake 配置：安装上列包后重新构建。
- SDK 下载失败：驱动 CMake 会按 CPU 架构下载 AUBO SDK `0.27.0-rc.4`，需可访问 `download.aubo-robotics.cn`；不能拿 x86_64 SDK 用于 ARM64。

最终构建结果：`7 packages finished`，驱动、控制器、消息、模型与规划包全部编译通过；
`aubo_gazebo` 按上述流程跳过。构建日志：`/tmp/aubo-build.log`（临时文件）。

本次同步修正：

- `ros_joints_plan` 删除 ROS 1 的 `tf` / `actionlib`，补齐 tf2、MoveIt 规划接口和 lint 测试依赖。
- `aubo_dashboard_msgs` 补齐 `builtin_interfaces`；驱动补齐 `ament_index_python` 和 CMake 对 `ament_cmake_python` 的显式查找。
- 硬件接口移除未使用的 `robot_mode` 消息头、`serviceinterface.h` 和 Jazzy 中不存在的 `visibility_control.h` 引用，显式包含 `unordered_map`。
- 已确认 `serviceinterface.h` 和 `aubo/robot/robot_state.h` 均由下载的 AUBO SDK 提供；前者在本硬件接口中未使用，移除以减少无关 SDK 依赖，后者保留。
- SDK 安装规则同时包含 Release 与 Debug 动态库，修复 Release 构建安装后缺少 `libaubo_sdk.so` 的问题。

验证：rosdep 检查通过（排除 Gazebo Classic 和 Docker）；驱动可执行文件可被 ROS 2 发现；
`aubo_control.launch.py --show-args` 解析成功；硬件插件的 `ldd` 无缺失库，`ctypes.CDLL` 加载成功。上述为早期编译检查，当时未连接机器人。
Jazzy 仍对旧式 `on_init(HardwareInfo)` 等接口给出弃用警告，不影响本次构建通过。

本小节仅记录早期编译和依赖检查；后续 ES3 真机结果见下文“ES 控制柜、实时性与频率验收”。Gazebo 仿真仍未验证。

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

首次运行先将 Python 依赖安装到当前用户环境，无需创建虚拟环境：

```bash
cd <your_ros2_ws>
python3 -m pip install --user --break-system-packages numpy 'pyaubo_sdk==0.27.1rc4'
python3 -c 'import numpy, pyaubo_sdk; print("校准依赖导入成功")'
```

上述 SDK 版本已在 ARM64 / Python 3.12 环境验证可导入。该平台需要使用带匹配 wheel 的预发布版本；直接执行 `pip install pyaubo_sdk` 可能提示找不到匹配版本。C++ SDK 不包含可直接导入的 Python 模块，仍需单独安装 Python SDK。安装方式参考 [AUBO 官方说明](https://developer.aubo-robotics.cn/guide_sdk/4_dev_preparation/2_installation.html)。

`--user` 将依赖安装到当前用户目录；Ubuntu 24.04 的系统 Python 使用 PEP 668 管理，因此这里通过 `--break-system-packages` 允许用户级安装。无需 `sudo pip`，也无需激活环境。如果终端此前已激活虚拟环境，请先执行 `deactivate`，再安装和运行。

推荐方法：

```bash
cd <your_ros2_ws>
source /opt/ros/jazzy/setup.bash
python3 src/aubo_ros2_driver/aubo_description/scripts/calibrate_urdf_dh.py \
  --robot-model aubo_i5 \
  --robot-ip 192.168.127.128
colcon build --packages-select aubo_description
source install/setup.bash
```

说明：

- 生成结果默认写入 `src/aubo_ros2_driver/aubo_description/urdf/<robot_model>_calibrated.urdf`。
- `--robot-ip` 需要显式传入。
- 运行前请确认当前 Python 环境可以导入 `numpy` 和 `pyaubo_sdk`。
- 生成后需要单独重新编译 `aubo_description` 包。

## 使用 aubo_ES3 校准模型运行真机

校准文件与具体机械臂对应。本机校准文件为 `aubo_description/urdf/aubo_ES3_calibrated.urdf`；采集时该机械臂使用 `172.16.31.105`，后续同一机械臂改为有线直连 `192.168.3.100`。请勿直接用于其他同型号机械臂。

首次使用新启动参数，或修改启动文件后，在工作空间根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select aubo_description aubo_controllers aubo_ros2_driver aubo_moveit_config \
  --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
source install/setup.bash
```

终端 1：启动真机驱动，加载校准模型：

```bash
cd ~/workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=aubo_ES3 robot_ip:=192.168.3.100 \
  use_fake_hardware:=false use_calibrated:=true launch_rviz:=false \
  controller_profile:=es servo_period:=auto servo_mode:=1
```

终端 2：启动 MoveIt 和 RViz，加载相同校准模型：

```bash
cd ~/workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch aubo_moveit_config aubo_moveit.launch.py \
  aubo_type:=aubo_ES3 use_calibrated:=true
```

两个入口都必须指定 `use_calibrated:=true`，`aubo_type` 保持原始型号，无需手动添加 `_calibrated`。该参数默认 `false`，保持已有启动方式；启用后缺少校准 URDF 会直接报错。MoveIt 使用 KDL 根据加载的 URDF 求解运动学。

再次校准时，给校准脚本传入 `--force` 可覆盖已有输出，然后执行 `colcon build --packages-select aubo_description`。重启驱动和 MoveIt，并在各终端重新 `source install/setup.bash`，使两者使用最新模型。启动时会按源 URDF 的更新时间刷新对应的 `.urdf.xacro`。

## ES 控制柜、实时性与频率验收

本节基于 2026-10-29 的 AGX Orin → ES3 有线直连实测，柜端设备树确认为 RK3568 / Forlinx OK3568-C。ROS 主机是 ARM 或 x86，不能据此推断远端控制柜的性能。以下频率结论只适用于本次柜体、固件和负载，不直接推广到其他 ES 或 x86 控制柜。

### 参数与默认值

| 参数 | 默认值 | 行为 |
| --- | --- | --- |
| `controller_profile` | `auto` | ES 型号选 100 Hz，其他型号沿用控制器 YAML；`es` 显式选 100 Hz，`legacy` 选 200 Hz，`custom` 沿用 YAML |
| `update_rate` | 空 | 显式指定时覆盖 profile，支持 25～500 Hz；这是控制器管理器的目标频率，不是硬实时保证 |
| `servo_period` | `0.01` | SDK `servoJoint` 的执行时间 `t`，单位秒；显式设置 `auto` 时按 `1/update_rate` 计算 |
| `servo_mode` | `1` | 进入的伺服模式；允许传入 SDK 定义的 1～7，柜端实际支持情况需另确认；0 仅供内部退出使用 |

SDK 建议 `t` 与连续调用 `servoJoint` 的间隔一致。修改频率时可显式使用 `servo_period:=auto`，并测量真实调用间隔；不要仅增大 `t` 来掩盖断流。自动配对也会改变 `5*t` 预算，不能消除同步 RPC 的长尾。驱动校验 `t` 为 0.002～0.02 秒且控制周期小于 `5*t`，因此部分低频与自动配对组合会被拒绝。

启动脚本在临时 YAML 中重写 `controller_manager` 节点专属的 `update_rate`，避免通配符参数不能覆盖原配置的问题。启动后检查日志中的目标频率和 `Cyclic timing`，也可查询：

```bash
ros2 param get /controller_manager update_rate
```

### 本次推荐候选：110 Hz 配对执行时间

ES 系列默认仍保留 100 Hz；本次这台柜体可显式采用以下已经过短时联合验收的配置：

```bash
cd ~/workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=aubo_ES3 robot_ip:=192.168.3.100 \
  use_fake_hardware:=false use_calibrated:=true launch_rviz:=false \
  controller_profile:=es update_rate:=110 servo_period:=auto servo_mode:=1
```

`110 Hz / t≈0.009090909 s / mode=1` 的实测结果：

| 项目 | 结果 |
| --- | --- |
| 有效观察窗口 | 约 180 秒，包含 wrist3 的 0.005 rad / 8 秒往返轨迹 |
| Servo 调用 | 19819 次，其中 1 次返回队列满 2，约 0.00505% |
| 超周期 | 1 条日志，记录循环约 10.89 ms |
| 柜端调用日志最大间隔 | 约 18.06 ms，低于本配置 `5*t≈45.45 ms` |
| 运行期退出 / 故障锁存 / RTDE 解析异常 | 均未出现 |
| 小轨迹结果 | 成功；反馈最大跟踪误差约 0.000449 rad，返回起点最大误差约 0.0000037 rad |

该组合通过本次“实际运行”判据，未通过“零超周期、零队列满”严格判据。它比 100 Hz 提高了 10%，但不是全局最优值或未来零故障保证；反馈误差也只是本次采样结果，不是完整工作空间的误差上界。

初筛中 120～124 Hz 的短时待机通过，不能代替联合验收：124 Hz 固定 `t=.01` 延长后出现队列满和超周期；120 Hz 自动配对时曾遇到约 40.8 ms RPC 长尾，触发缩短后的本地看门狗预算，工具停止该轮且未执行轨迹。不要只根据待机“没有掉线”提高频率。

详细结果见本工作区的[控制频率搜索与工具收敛报告](../../log/控制频率搜索与工具收敛报告.md)及[110 Hz 运行判据复核](../../log/rate110_matched_confirmation_20261029/runtime_assessment.json)。这些是本地测量产物，不随单独克隆驱动仓库提供。

### 实时性改动与验收口径

- 辅助控制器依次加载，运动控制器最后激活；前置 spawner 失败则结束启动。
- IO、robot_manage、robot_config 状态通过 `RealtimePublisher.try_publish` 交给后台线程发布，复用消息缓存；发布忙时跳过旧快照。
- 运动周期内不逐次查询伺服模式，队列满时不循环休眠重试；保留返回码时间预算和故障锁存。
- 关节/模式 RTDE 为 200 Hz，TCP、IO、payload 等扩展状态为 50 Hz；这与控制器管理器的目标频率是不同设置。
- 真实 IO / 配置命令仍同步执行，SDK 单次 RPC 仍可能阻塞；本次没有覆盖这些服务与运动并发的负载。

工程验收区分两层：

| 判据 | 用途 |
| --- | --- |
| `strict` | 要求观测窗口内零超周期、零队列满，用于比较周期确定性 |
| `runtime` | 允许少量超周期及暂时返回 2/3，重点检查 Servo 连续、无故障/运行期自动退出、反馈完整，并验证所请求轨迹成功 |

`5*t` 不退出是运行底线，不能单独证明轨迹跟踪合格。队列满不表示新目标被接受，仍需同时检查返回码、指令间隔和轨迹误差。当前 `5*t` 策略以模式 1 的现场记录为依据，其他模式需核对柜端语义。上述判据不会关闭警告、放宽轨迹容差或取消故障保护。

FIFO 权限检查入口见下一节。本机已确认 RT 内核及控制线程 `FF / priority 50`，但 FIFO 调度不能消除网络、SDK 或柜端的阻塞。校准版 ES3 的微幅单轴验证不代表高负载、多轴轨迹、全部 IO 功能或 x86 控制柜已验收。

## 本地诊断工具

本工作区统一入口为 `scripts/aubo_diag.py`，完整说明见[本地诊断工具 README](../../scripts/aubo_diagnostics/README.md)。工具位于工作区级 `scripts/`，不在驱动仓库内部；单独克隆本仓库不会包含它们。历史 `log/` 中的试验脚本仅作为原始证据保留，后续复用以统一入口为准。

```bash
cd ~/workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/aubo_diag.py --help
```

| 子命令 | 用途 |
| --- | --- |
| `fifo` | 检查 RT 内核、rtprio/memlock 和实际 FF 线程；默认只读，`--apply` 才修改权限配置 |
| `network` / `rpc` | 路由、TCP、Ping、可选 FIFO 唤醒延迟，以及 SDK 只读查询耗时 |
| `collect` | 通过 SSH 按控制柜本地时间提取轮转日志 |
| `sweep` | 分档启动/停止真机驱动，采集双端日志，按所选判据验收 |
| `observe` | 被动观察关节状态到达间隔、状态话题数量及关节行程 |
| `motion` | 已启动驱动上的受限 wrist3 往返验收，不自动启动驱动 |
| `report` | 合并已有结果并生成 Markdown 与候选配置 JSON，不改写原始判定 |

先预览扫频计划，不连接或启动硬件：

```bash
python3 scripts/aubo_diag.py sweep --rates 100,110 \
  --servo-period auto --servo-mode 1 --acceptance runtime \
  --output log/my_rate_plan
```

确认现场已准备、没有重复驱动或外部运动程序后，再执行联合验收：

```bash
# 密码仅放入当前 shell 环境，不写入脚本或可共享命令历史。
read -rsp '控制柜 SSH 密码: ' AUBO_SSH_PASSWORD; echo
export AUBO_SSH_PASSWORD
python3 scripts/aubo_diag.py sweep --run --rates 110 \
  --servo-period auto --servo-mode 1 --acceptance runtime \
  --micro-trajectory --warmup 15 --duration 180 \
  --robot-ip 192.168.3.100 --ssh-port 8822 --output log/rate110_runtime_01
unset AUBO_SSH_PASSWORD
```

默认只显示计划；`--run` 启动真机待机 Servo，另加 `--micro-trajectory` 才会在观察期内执行同一条 8 秒微幅轨迹。独立 `motion` 子命令需要 `--run-motion`。微轨迹工具不提供 MoveIt 避障或工具环境碰撞验证。

每档保存原始驱动/柜端日志、启动 PID、检查点及最终 `summary.json`；请使用新的输出目录。硬件故障立即停止整轮，不自动恢复。报告按模式、Hz、t 和判据区分；同配置后续失败会否决此前短测候选，缺失证据和未完成记录不判通过。柜端时间可能与 Orin 有偏移，不能直接相减当作单向网络延迟。

网络与 FIFO 默认检查不运动；SDK 只读探针另从 `AUBO_RPC_PASSWORD` 读取 SDK 密码，不应混用 SSH root 密码。工具自检：

```bash
python3 -m unittest discover -s scripts/aubo_diagnostics/tests -v
```

## 驱动真实机械臂 aubo_i5（修改机器人对应 `robot_ip`、`aubo_type`）

```bash
source install/setup.bash
ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5 robot_ip:=192.168.127.128 \
  use_fake_hardware:=false
ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=aubo_i5
```

在 ROS 2 Jazzy 下，`aubo_moveit.launch.py` 已完成基础适配，可直接用于 RViz + MoveIt 联调。`launch_rviz` 默认为 `true`。

### 伺服模式参数

驱动在运动控制器下发 `position` 关节目标时进入 SDK 伺服运动模式，用于持续执行 `servoJoint` 控制；运动控制器停止后会退出伺服模式。本驱动使用 `MotionControl.setServoModeSelect()` / `getServoModeSelect()` 进入和确认伺服模式，不再使用旧的 `setServoMode(true/false)` 接口。

`aubo_control.launch.py` 通过 `servo_mode` → Xacro → hardware 参数传递进入模式，默认值为 `1`。此前参数未写入 hardware 的链路缺口已修复；本次真机结果全部由日志确认实际为模式 1。其他模式仅验证参数传递，尚未进行运动对照。

`common_interface` 中定义的取值如下：

| `servo_mode` | 含义 |
| --- | --- |
| `0` | 退出伺服模式，仅供驱动内部停用使用；不能作为活动启动模式 |
| `1` | 截断式规划伺服模式，默认进入模式 |
| `2` | 透传模式，直接下发 |
| `3` | 透传模式，缓存下发 |
| `4` | SDK 定义的 1 ms 缓存透传模式；需确认柜端支持，不代表 ROS/RPC 可稳定 1000 Hz |
| `5` | 规划伺服模式 |
| `6` | 截断式规划伺服模式，可以叠加 SDK `ForceControl` 力控 |
| `7` | 规划伺服模式，可以叠加 SDK `ForceControl` 力控 |

本驱动默认使用已验证的模式 1。模式 1 更新目标和规划路线，不保证经过旧目标；模式 5 要求经过所有目标点，因此排队和轨迹时序可能不同。2/3 的直接或缓存透传改变了客户端与柜端的职责，6/7 还涉及力控；不能只为减少队列满而切换模式。

- SDK 建议 `servoJoint` 的 `t` 与实际下发间隔一致，可显式使用 `servo_period:=auto` 配对目标频率，再检查真实调用时序。
- 当前 ros2_control 写循环每周期最多尝试一次；返回 2/3 时不会忙等或重放旧点，而由时间预算和后续新目标处理。
- 若离线任务必须经过每个目标点，应在控制循环之外处理重采样、限速和队列，不能把无限重试移入实时 `write()`。
- 比较频率时必须固定模式；换模式后，调用间隔、队列行为、超时语义和轨迹误差都需要重新验证。

不同控制器和 SDK 版本对模式支持可能存在差异，实际可用取值请以当前控制器配套 SDK 文档为准。

当前运动命令链路支持 `position` command interface。`velocity` 仅保留接口占位，`speedServo()` 后端尚未实现；激活后会报错并进入故障处理，不应将其用于真机速度控制。

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

`/io_control/state`、`/robot_manage/state` 和 `/robot_config/state` 每个控制周期尝试向 `RealtimePublisher` 投递最新快照，实际 DDS 发布由后台线程执行。忙时会跳过旧快照，实际话题频率可能低于 `controller_manager.update_rate`；扩展 RTDE 数据源仍为 50 Hz。

实时控制、RTDE 状态、IO、Payload、TCP offset 与拖动示教仍收敛在 `AuboHardwareInterface` 和 ros2_control controllers。机器人上电、启动、断电、松刹车、解除保护停等生命周期能力由 `dashboard_client` 通过 9012 WebSocket JSON-RPC 提供；这些操作可能改变机器人运行状态。进入 Servo 会话前状态不就绪时驱动会暂停输出；已进入会话后离开可运动状态或触发通信保护时，会锁存故障并拒绝继续旧会话。状态恢复不会自动清除故障，需要按实际硬件生命周期显式恢复，再提交新轨迹。`aubo_controllers` 不连接 SDK，也不订阅 RTDE。`controller_stopper` 只根据 `/robot_manage/state` 编排运动控制器启停。

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

在 ROS 2 Jazzy 下，`ros_joints_plan` 的 launch 配置也已同步到当前 MoveIt 适配方案。

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
