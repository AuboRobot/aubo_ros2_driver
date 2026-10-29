# 对齐 aubo_dual_ros2_driver 的可借鉴项（Humble / 单臂）

来源：`~/aubo_dual_ros2_driver`（gitee `aubo_bipt/aubo_dual_ros2_driver` @ master）。  
本仓库分支：`dual-align-arcs`（基于 `humble-port`）。

## 功能矩阵

| 能力 | dual | 本仓库（移植后） | 说明 |
|---|---|---|---|
| 启动判断自动上电 | `ensureArmRunning` @ `OnActive` | `ensureRobotRunning` @ `OnActive` | 单臂；已 Running 跳过；PowerOff→poweron→startup |
| 解除防护停止 | 按臂 Trigger 服务 + 激活前清理 | 激活前 `clearProtectiveStopIfNeeded`；运行时仍用 `/robot_manage/unlock_protective_stop` | 未移植双臂 GUI 左右按钮 |
| 编译自动 setcap FIFO RT | CMake `apply_aubo_rt_caps` ALL + scripts | 同模式，二进制 `aubo_ros2_control_node` | 首次需 `configure_aubo_realtime.sh`（sudo） |
| 多机 remote RViz | `remote_moveit_rviz.launch.py` + `remote_viz` | `aubo_moveit_config/launch/remote_moveit_rviz.launch.py`；`aubo_control.launch.py` 增加 `remote_viz` | 控制端 `remote_viz:=true`，可视化机另起 RViz |
| 模式切换 SERVO/NORMAL/TEACH | `aubo_mode_manager` | `scripts/switch_mode.sh` 文档型辅助；TEACH 未移植 freedrive 服务 | 双臂 mode_manager 不直接搬 |

## ARCS sim-as-real（本机当真机调试）

- ARCS 保持 `hw = "fake_robot1"`、`robot_type = "aubo_ES3"`（提供本机 30004/30010/9012）。
- **`robot_ip` 必须 `127.0.0.1`**，不要填局域网 IP。
- `robot_driver1` 仅在有真实控制柜/本体线缆时使用，与本路径不同。

```bash
# ARCS 已 active 时：
source /opt/ros/humble/setup.bash
export AUBO_SDK_ROOT=$HOME/ws/aubo/aubo_sdk-0.27.0-rc.4-Linux_x86_64+b7ed04d
source ~/ws_humble/install/setup.bash
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=aubo_ES3 robot_ip:=127.0.0.1 \
  use_fake_hardware:=false use_calibrated:=false \
  launch_rviz:=false remote_viz:=true
```

解除防护（dashboard）：

```bash
ros2 service call /robot_manage/unlock_protective_stop std_srvs/srv/Trigger {}
# 或
bash $(ros2 pkg prefix aubo_ros2_driver)/lib/aubo_ros2_driver/switch_mode.sh UNLOCK
```

setcap（每台机器一次 + 编译后自动补）：

```bash
# 安装受限 wrapper 并配置 sudoers（需交互/密码）
bash ~/repos/aubo_ros2_driver/aubo_ros2_driver/scripts/configure_aubo_realtime.sh
# 运行中检查
bash ~/repos/aubo_ros2_driver/aubo_ros2_driver/scripts/check_aubo_rt.sh
```

## 未搬迁

- 双臂 `aubo_mode_manager` 整包、GUI 解除防护按钮、双臂 hardware 插件。
- dual 的 S100/`/mnt/ssd` nosuid 专项（脚本已保留检测逻辑，x86 PC 通常不需要）。

## Diagnostics servo_mode（频率联调）

与上表「模式切换 SERVO/NORMAL/TEACH」不同：`aubo_control.launch.py` 的 `servo_mode:=N` / `aubo_diag.py sweep --servo-mode N` 对应 SDK `setServoModeSelect`（1–7）。
本机假机频率矩阵与 200Hz 规划/透传差异见 develop-workflows：`env_deploy/arcs/LOCAL_DIAG.md`。
