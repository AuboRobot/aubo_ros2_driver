# aubo_ros2_driver

遨博机器人ROS2驱动

## 安装与编译（Ubuntu 22.04 / ROS 2 Humble）

先建立 ROS 工作区，再把本仓库克隆到 `src` 下。克隆完成后目录为 `~/ws_humble/src/aubo_ros2_driver`，各个 ROS 包在该目录内。编译、校准和启动命令都在工作区根目录 `~/ws_humble` 执行。模型由 Git 子模块提供，首次克隆后必须初始化：

```bash
mkdir -p ~/ws_humble/src
cd ~/ws_humble/src
git clone https://github.com/AuboRobot/aubo_ros2_driver.git
cd ~/ws_humble
git -C src/aubo_ros2_driver submodule update --init --recursive
source /opt/ros/humble/setup.bash
# 首次使用 rosdep 时按需执行 sudo rosdep init；已有配置无需重复初始化
rosdep update
rosdep install --from-paths src/aubo_ros2_driver --ignore-src -r -y --rosdistro humble \
  --skip-keys="gazebo_ros gazebo_ros2_control docker.io"
CMAKE_BUILD_PARALLEL_LEVEL=4 colcon build --base-paths src/aubo_ros2_driver --packages-skip aubo_gazebo \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`aubo_gazebo` 依赖 Gazebo Classic；上述 Humble 编译流程跳过此包及其两项依赖，不代表已验证 Gazebo 仿真。
`--base-paths src/aubo_ros2_driver` 只构建这次克隆进来的仓库。之后若在 `src` 下再放入其他仓库，它们不会被这次命令编进去。
内存较少时可将 `CMAKE_BUILD_PARALLEL_LEVEL` 降为 `2`。Docker 仅为测试依赖，本次未安装；需要 Docker 测试时移除对应 skip key。
请以普通用户运行 rosdep，按提示使用 sudo 安装系统包；不要直接用 root 运行 rosdep，否则可能找不到用户的 rosdep 缓存。

### 本机依赖检查与排查记录（2026-10-29）

已执行实际 colcon 构建及 `rosdep check --from-paths src/aubo_ros2_driver --ignore-src`。
初始环境缺少以下系统依赖（安装操作需要 sudo）：

```bash
sudo apt update
sudo apt install \
  ros-humble-controller-manager ros-humble-controller-manager-msgs \
  ros-humble-hardware-interface ros-humble-controller-interface ros-humble-realtime-tools \
  ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller \
  ros-humble-position-controllers ros-humble-velocity-controllers \
  ros-humble-force-torque-sensor-broadcaster ros-humble-ros2-controllers-test-nodes \
  ros-humble-control-msgs ros-humble-moveit-msgs \
  ros-humble-moveit-ros-move-group ros-humble-moveit-kinematics ros-humble-moveit-planners \
  ros-humble-moveit-simple-controller-manager ros-humble-moveit-configs-utils \
  ros-humble-moveit-ros-visualization ros-humble-moveit-ros-warehouse \
  ros-humble-moveit-setup-assistant ros-humble-moveit-ros-planning-interface \
  ros-humble-warehouse-ros ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-xacro nlohmann-json3-dev socat
```

以上为 Humble 依赖名；完整安装以初始化子模块后的 rosdep 结果为准。
此前环境安装时 apt 新增 101 个包，其中包括驱动运行依赖 `socat`。
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
- 硬件接口移除未使用的 `robot_mode` 消息头、`serviceinterface.h` 和 `visibility_control.h` 引用，显式包含 `unordered_map`。
- 已确认 `serviceinterface.h` 和 `aubo/robot/robot_state.h` 均由下载的 AUBO SDK 提供；前者在本硬件接口中未使用，移除以减少无关 SDK 依赖，后者保留。
- SDK 安装规则同时包含 Release 与 Debug 动态库，修复 Release 构建安装后缺少 `libaubo_sdk.so` 的问题。

验证：rosdep 检查通过（排除 Gazebo Classic 和 Docker）；驱动可执行文件可被 ROS 2 发现；
`aubo_control.launch.py --show-args` 解析成功；硬件插件的 `ldd` 无缺失库，`ctypes.CDLL` 加载成功。上述为早期编译检查，当时未连接机器人。
旧式 `on_init(HardwareInfo)` 等接口可能给出弃用警告，不影响本次构建通过。

本小节仅记录早期编译和依赖检查；后续 ES3 真机结果见下文“ES 控制柜、实时性与频率验收”。Gazebo 仿真仍未验证。

## ros2_control 内核实时性配置

控制器管理器会在主控制循环上申请优先级 50 的 `SCHED_FIFO`。普通用户默认不能设置该调度策略。文件 capability 让 `aubo_ros2_control_node` 在不开放通用 root 的前提下获得 `CAP_SYS_NICE` 和 `CAP_SYS_RESOURCE`。

### 配置实时权限

这是不依赖 file capability 的备选：把当前用户加入 `realtime` 组并放宽 `rtprio`。改完后必须重新登录。

**步骤 1：创建 realtime 用户组**

```bash
sudo addgroup realtime
sudo usermod -a -G realtime $(whoami)
```

**步骤 2：配置系统限制**

编辑 `/etc/security/limits.conf`，在文件末尾为 realtime 组添加：

```
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock unlimited
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock unlimited
```

**步骤 3：重新登录**

限制在新的登录会话里生效。可用 `ulimit -r` 查看当前 `rtprio` 上限。

### 关于实时调度

- 实时优先级范围是 0–99，数值越大优先级越高。本驱动控制环使用 50。
- `SCHED_FIFO` 用于控制环；桌面和其他普通进程仍是 `SCHED_OTHER`。
- 控制面与控制柜保持同一二层有线直连。FIFO 不能消除网络、SDK 或整机卡死造成的控制环停顿；柜端约 `5*t` 收不到成功 `servoJoint` 会 Automatic exit。频率与链路的验收口径见下文“ES 控制柜、实时性与频率验收”。

调度策略说明见 [sched(7)](https://man7.org/linux/man-pages/man7/sched.7.html)。

### 受限的 setcap 自动配置

不要给普通用户开放无参数限制的 `sudo setcap`。那会允许对任意程序授予 `cap_setuid`、`cap_sys_admin` 等高危 capability，效果接近 root。

`aubo_ros2_driver/scripts/apply_aubo_rt_caps.sh` 由 CMake 的 `apply_aubo_rt_caps` ALL 目标在每次 `colcon build` 时调用，不依赖是否重新链接。真机启动前也会再执行一次。它给 `aubo_ros2_control_node` 补 `cap_sys_nice,cap_sys_resource+ep`。两条环境的免密范围不要混用：

| 环境 | 免密 sudo | 编译期行为 |
| --- | --- | --- |
| PC x86 | 仅 `/usr/local/sbin/aubo-setcap-rt` | 只调用该包装器。包装器设置 capability，并在缺路径时把本工作区的消息类型支持库目录追加到 `/etc/ld.so.conf.d/aubo_ros2_rt.conf`，然后执行 `ldconfig`。不调用 `sudo -n cp`、`sudo -n setcap` 或 `sudo -n ldconfig`，因此 colcon 不应出现 `sudo: 需要密码` |
| RDK S100 aarch64 | `sunrise` 常有 `NOPASSWD: ALL` | 可以不装包装器，直接 `sudo -n /usr/sbin/setcap`。`apply_aubo_rt_caps.sh` 在需要时写入同一个 conf。S100 重启会丢掉 file capability，所以每次编译和真机启动都会再打 |

包装器只接受 `*/aubo_ros2_driver/build/` 或 `*/ws_humble/build/` 下、且归当前用户所有的可执行文件，capability 字符串固定。`--symlink-install` 时 install 树里的节点是指向 build 目录真实二进制的符号链接，`setcap` 作用在该真实文件上。

file capability 会使进程进入 `AT_SECURE`，动态链接器忽略 `LD_LIBRARY_PATH`。插件自己的 `RUNPATH` 只能解析它的 `NEEDED` 依赖。`io_and_status_controller` 创建 `aubo_msgs` publisher 时，`rmw` 还会按库名加载 `libaubo_msgs__rosidl_typesupport_introspection_cpp.so`。这个库不在插件的 `NEEDED` 列表里，因此工作区 `install/*/lib` 中带 `rosidl_typesupport` 的目录必须进入 `ld.so.cache`。缺这条路径时，控制器 configure 失败，日志里是 `type_support is null` 和 `cannot open shared object file`。

包装器只追加从该二进制推导出的目录：ROS `lib`、multiarch 子目录、`install/aubo_ros2_driver/lib`，以及 `install/*/lib` 里实际含有 `lib*__rosidl_typesupport_*.so` 的目录。已有行保留，路径齐全时不再执行 `ldconfig`。

**PC x86 首次配置**（尚未安装包装器、或更新了 `aubo_setcap_rt_wrapper.sh` 之后）：

```bash
cd ~/ws_humble
sudo install -o root -g root -m 0755 \
  src/aubo_ros2_driver/aubo_ros2_driver/scripts/aubo_setcap_rt_wrapper.sh \
  /usr/local/sbin/aubo-setcap-rt
echo "$(whoami) ALL=(root) NOPASSWD: /usr/local/sbin/aubo-setcap-rt" | sudo tee /etc/sudoers.d/aubo-setcap-rt >/dev/null
sudo chmod 440 /etc/sudoers.d/aubo-setcap-rt
sudo visudo -cf /etc/sudoers.d/aubo-setcap-rt
```

`$(whoami)` 由当前 shell 展开后再写入文件，sudoers 里保存的是当前用户名。`visudo -cf` 只检查语法，通过时退出码为 0。

仓库里的脚本更新后，必须重新执行上面的 `sudo install`。`/usr/local/sbin/aubo-setcap-rt` 不会随 `colcon build` 自动替换。

然后确认免密包装器可以同时补上 capability 和动态库缓存：

```bash
cd ~/ws_humble
source /opt/ros/humble/setup.bash
source install/setup.bash
sudo -n /usr/local/sbin/aubo-setcap-rt \
  build/aubo_ros2_driver/aubo_ros2_control_node
getcap build/aubo_ros2_driver/aubo_ros2_control_node
grep aubo_msgs /etc/ld.so.conf.d/aubo_ros2_rt.conf
```

`getcap` 应包含 `cap_sys_nice,cap_sys_resource=ep`。conf 中应有 `install/aubo_msgs/lib`。之后 `colcon build` 会自动补 capability；若又出现新的工作区消息包，同一次编译会把对应 `lib` 追加进 conf。

`src/aubo_ros2_driver/aubo_ros2_driver/scripts/configure_aubo_realtime.sh` 仍用于检查挂载是否带 `nosuid`（该选项会让 file capability 的 `CapEff` 保持为 0），并在已 source 工作区时复检类型支持库能否在不设置 `LD_LIBRARY_PATH` 的情况下按库名加载。挂载含 `nosuid` 时 FIFO 不会生效，需先改挂载选项再重新登录或 remount。

## ROS 2 Humble 适配状态

当前仓库按 ROS 2 Humble 使用。`aubo_ros2_driver`、`aubo_description`、`aubo_moveit_config` 可完成基础启动、状态显示、MoveIt 规划与轨迹执行。

本次适配已处理的关键问题包括：

- MoveIt OMPL 使用 Humble 参数格式：`planning_plugin` 与 `request_adapters` 为字符串，插件名为 `default_planner_request_adapters/*`。不要改回字符串数组或 `default_planning_*` 插件名，否则 `move_group` 会因参数类型不匹配退出。
- `realtime_tools::RealtimePublisher` 使用 Humble 的 `tryPublish`。
- MoveIt 控制器管理器参数补齐，支持 `joint_trajectory_controller` 执行。
- `joint_limits.yaml` 补充加速度限制，修复 `AddTimeOptimalParameterization` 失败问题。
- `SRDF` 虚拟关节修正为 `world -> base_link`。
- RViz `Planning Scene Topic` 使用 `move_group/monitored_planning_scene`，与 `aubo_moveit.launch.py` 中的 planning scene monitor 一致。

说明：

- 当前文档中的 MoveIt 相关启动方式，均以 Humble 配置为准。
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
cd ~/ws_humble
python3 -m pip install --user numpy 'pyaubo_sdk==0.27.1rc4'
python3 -c 'import numpy, pyaubo_sdk; print("校准依赖导入成功")'
```

Python SDK 曾在 ARM64 / Python 3.12 环境验证可导入。当前 Humble 环境是 Ubuntu 22.04 / Python 3.10，仍需使用带匹配 wheel 的预发布版本；直接执行 `pip install pyaubo_sdk` 可能提示找不到匹配版本。C++ SDK 不包含可直接导入的 Python 模块，仍需单独安装 Python SDK。安装方式参考 [AUBO 官方说明](https://developer.aubo-robotics.cn/guide_sdk/4_dev_preparation/2_installation.html)。

`--user` 将依赖安装到当前用户目录。Ubuntu 22.04 一般不需要 `--break-system-packages`。无需 `sudo pip`，也无需激活环境。如果终端此前已激活虚拟环境，请先执行 `deactivate`，再安装和运行。

推荐方法：

```bash
cd ~/ws_humble
source /opt/ros/humble/setup.bash
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

首次使用新启动参数，或修改启动文件后，在 `~/ws_humble` 执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select aubo_description aubo_controllers aubo_ros2_driver aubo_moveit_config \
  --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
source install/setup.bash
```

终端 1：启动真机驱动，加载校准模型：

```bash
cd ~/ws_humble
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=aubo_ES3 robot_ip:=192.168.3.100 \
  use_fake_hardware:=false use_calibrated:=true launch_rviz:=false \
  controller_profile:=es servo_period:=auto servo_mode:=1
```

终端 2：启动 MoveIt 和 RViz，加载相同校准模型：

```bash
cd ~/ws_humble
source /opt/ros/humble/setup.bash
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
cd ~/ws_humble
source /opt/ros/humble/setup.bash
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
- IO、robot_manage、robot_config 状态通过 `RealtimePublisher.tryPublish` 交给后台线程发布，复用消息缓存；发布忙时跳过旧快照。
- 运动周期内不逐次查询伺服模式，队列满时不循环休眠重试；保留返回码时间预算和故障锁存。
- 关节/模式 RTDE 为 200 Hz，TCP、IO、payload 等扩展状态为 50 Hz；这与控制器管理器的目标频率是不同设置。
- 真实 IO / 配置命令仍同步执行，SDK 单次 RPC 仍可能阻塞；本次没有覆盖这些服务与运动并发的负载。

工程验收区分两层：

| 判据 | 用途 |
| --- | --- |
| `strict` | 要求观测窗口内零超周期、零队列满，用于比较周期确定性 |
| `runtime` | 允许少量超周期及暂时返回 2/3，重点检查 Servo 连续、无故障/运行期自动退出、反馈完整，并验证所请求轨迹成功 |

`5*t` 不退出是运行底线，不能单独证明轨迹跟踪合格。队列满不表示新目标被接受，仍需同时检查返回码、指令间隔和轨迹误差。当前 `5*t` 策略以模式 1 的现场记录为依据，其他模式需核对柜端语义。上述判据不会关闭警告、放宽轨迹容差或取消故障保护。

本机已确认 RT 内核及控制线程 `FF / priority 50`，但 FIFO 调度不能消除网络、SDK 或柜端的阻塞。校准版 ES3 的微幅单轴验证不代表高负载、多轴轨迹、全部 IO 功能或 x86 控制柜已验收。FIFO file capability 与 `ld.so.conf` 的配置见上文“ros2_control 内核实时性配置”。

## 驱动真实机械臂 aubo_i5（修改机器人对应 `robot_ip`、`aubo_type`）

```bash
source install/setup.bash
ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5 robot_ip:=192.168.127.128 \
  use_fake_hardware:=false
ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=aubo_i5
```

在 ROS 2 Humble 下，`aubo_moveit.launch.py` 可直接用于 RViz + MoveIt 联调。`launch_rviz` 默认为 `true`。

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

在 ROS 2 Humble 下，`ros_joints_plan` 的 launch 配置使用同一套 MoveIt 参数。

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

上电命令按当前模式处理，等待上限为 `mode_timeout_seconds`：已经 Running 则跳过；正在下电则先等到 PowerOff、Idle 或 Running；只有 PowerOff 才调用 `poweron`（返回码 0 或 `AUBO_BUSY` 都算接受）；刹车松开中则等待 Running；其余已上电状态跳过 `poweron`，需要时再 `startup` 直到 Running。无控制器、急停或故障会直接失败：

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
