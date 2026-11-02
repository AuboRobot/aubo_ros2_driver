# ES 控制柜与频率验收

本文只记录 ES 控制柜及一台 ES3 的现场结果。全系列的启动参数、实时权限和伺服模式见仓库 [README](../../README.md)。

本节基于 2026-10-29 的 AGX Orin → ES3 有线直连实测，柜端设备树确认为 RK3568 / Forlinx OK3568-C。ROS 主机是 ARM 或 x86，不能据此推断远端控制柜的性能。以下频率结论只适用于本次柜体、固件和负载，不直接推广到其他 ES 或 x86 控制柜。

## 使用 aubo_ES3 校准模型运行真机

校准文件与具体机械臂对应。本机校准文件为 `aubo_description/urdf/aubo_ES3_calibrated.urdf`，对应有线直连 `robot_ip:=192.168.3.100` 的机械臂。请勿直接用于其他同型号机械臂。`use_calibrated` 的通用规则见 README 的 URDF 校准一节。

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

## 碰撞后手动恢复与 ROS 自动重入 Servo（2026-11-02）

在 aubo-3060 / ROS 2 Humble 上，对 ES3 进行了 ROS 层真机恢复验证。本次连接地址为 `172.16.31.105`，使用校准模型，参数为 `controller_profile:=es servo_period:=auto servo_mode:=1`；启动日志确认控制周期和 Servo 执行时间均为 10 ms（100 Hz）。地址变化不代表校准文件可跨机械臂复用。

现场确认：碰撞中断后，保持驱动运行、不按 `Ctrl+C`，由操作者完成手动上电及启动恢复。机器人重新达到 `Running` 且安全状态为 `Normal`（或 `ReducedMode`）、反馈新鲜后，驱动自动回到 Servo 模式 1；无需手动 switch mode，也无需重启 ROS 驱动。已中止的目标不会续跑，用户重新提交的新目标可以成功执行。

日志批次为 `2026-11-02-10-53-53-405776-aubo-3060-108612`。以下为 ROS 主机时间：

- 10:55:03.215：执行中的轨迹报告 `AUBO motion paused by safety state or unavailable feedback; submit a new trajectory after recovery`；硬件接口报告 `robot_mode_: 8, safety_mode_: 5`，暂停运动输出。
- 10:55:37.725：手动恢复后打印 `Robot returned to motion-ready state...`。
- 10:55:37.741：打印 `Servo mode select 1 enabled.`，确认自动重新进入 Servo。
- 10:56:05.966：接收并接受新的 action goal；10:56:07.845 打印 `Goal reached, success!`。

此前 10:46 批次还从 MoveIt 日志确认，碰撞后结果为 `PATH_TOLERANCE_VIOLATED / ABORTED`，不再误报 `SUCCEEDED`。两个批次分别覆盖 action 中止传播，以及保持驱动运行后的 Servo 恢复和新目标执行。

用户恢复步骤和 `read → controller update → write` 机制见 [README：碰撞或安全暂停后的 ROS 恢复](../../README.md#碰撞或安全暂停后的-ros-恢复)。自动恢复的前提是控制器激活、反馈新鲜且没有 `Servo fault latched`；驱动不自动上电、不自动解除保护停，也不绕过柜端状态。

本次只验收 ROS 层的暂停、中止、Servo 重入及新目标执行，不分析柜端断电原因，也不代表其他型号、固件或跟踪容差已全部验收。同一日志早期还出现过一次 53.963 ms 调用间隔及短时模式切换返回 `-3`，随后自动重入成功；因此本次恢复通过不等同于零超时或零警告验收。

## 本次推荐候选：110 Hz 配对执行时间

ES 的 100 Hz 配置通过 `controller_profile:=es` 显式选择；本次这台柜体可显式采用以下已经过短时联合验收的配置：

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

详细结果见本工作区的[控制频率搜索与工具收敛报告](../../../../log/控制频率搜索与工具收敛报告.md)及[110 Hz 运行判据复核](../../../../log/rate110_matched_confirmation_20261029/runtime_assessment.json)。链接按 README 的安装布局 `~/ws_humble/src/aubo_ros2_driver` 计算。这些是本地测量产物，不随单独克隆驱动仓库提供。

## 本次验收边界

真实 IO / 配置命令仍同步执行，SDK 单次 RPC 仍可能阻塞。本次没有覆盖这些服务与运动并发的负载。

当前 `5*t` 策略以模式 1 的现场记录为依据，其他模式需核对柜端语义。判据定义见 README“控制频率与伺服周期”；上述判据不会关闭警告、放宽轨迹容差或取消故障保护。

本机已确认 RT 内核及控制线程 `FF / priority 50`，但 FIFO 调度不能消除网络、SDK 或柜端的阻塞。校准版 ES3 的微幅单轴验证不代表高负载、多轴轨迹、全部 IO 功能或 x86 控制柜已验收。FIFO file capability 与 `ld.so.conf` 的配置见 README“ros2_control 内核实时性配置”。
