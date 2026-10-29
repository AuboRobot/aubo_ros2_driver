# ES3 真机 Servo 日志优化迁移（2026-10-29）

## 依据与适配

参考工作区 `learn/aubo_dual_ros2_driver`：

- `notebook/s100_servo_optimization_20261010.md`：故障锁存、显式恢复、残留模式清理；有线待机和残留模式清理已有现场验证，完整轨迹仍需验证。
- `notebook/s100_servo_log_review_20261009.md`：同步 RPC 长尾与逐调用诊断。
- `notebook/s100_moveit_execute_timeout_servoj_t.md`：增大 t=.02 曾造成轨迹跟踪超时。
- `src/aubo_dual_hardware/src/aubo_dual_hardware_interface.cpp`：单次发送、steady clock 时间预算、禁止自动补模式。

适配当前 SDK 的 `setServoModeSelect` API；保留本项目 200Hz、t=.01、gain/lookahead/a/v 和 JTC 容差。learn 最终采用过 100Hz 与 t=.01 对齐，但不同平台的频率结论不直接当成 ES3 验证结果。本次不改变轨迹动态参数。

## 已迁移

- Servo 已启动时使用本地会话状态，不在每个周期 getServoModeSelect。
- 初次进入模式跨周期推进（返回1表示pending）；150ms期限内每周期最多查询一次，不循环sleep。同步RPC本身仍可阻塞。
- servoJoint 每周期只调用一次，复用6维发送缓存；队列满不重发历史点、不sleep。
- 仅0表示当前目标被接受；2/3共用150ms待定预算，-5使用25ms预算。两类窗口独立，只由0或显式恢复清除；-13及其他异常立即锁存。
- 返回码错误/异常不再被write吞掉；返回hardware ERROR。锁存后拒绝继续运动，不自动重进伺服。该机制不是急停，已经发到控制柜的RPC无法取消。
- 记录RPC耗时、相邻尝试起点间隔和返回码。首条异常立即输出，普通异常5s节流；故障不受节流影响。单次RPC或相邻尝试间隔>=50ms（当前5*t）拒绝继续旧会话，即使RPC迟到返回0也锁存。这是保守的本机判据，不证明柜端精确收包时间。
- 激活等待真实RTDE反馈（最多2s），要求反馈年龄<100ms且各关节速度绝对值<.02rad/s；清理残留模式后再次检查并用实测位置初始化指令。
- 高频通道0仅发送关节/模式（200Hz），扩展TCP/IO/payload使用独立通道1（50Hz）；两者都检查topic/subscription返回值。关节包先完整解析、校验6轴及有限数，再一起提交并更新接收时间；过期>100ms锁存。
- 无IO/配置命令时直接返回，避免每周期生成辅助参数缓存与获取SDK接口。移除启动时写入255/44/3.1/4.1等演示寄存器值，以及无消费者的200Hz机器人消息订阅。
- 修正三个关节的Xacro限位表达式；未启用enforce_command_limits，未改变已有范围。

## 本地验证（不连接机器人）

```bash
cd ~/workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-select aubo_description aubo_ros2_driver \
  --cmake-args -DBUILD_TESTING=ON -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
colcon test --packages-select aubo_ros2_driver --ctest-args -R test_servo_return_policy
colcon test-result --test-result-base build/aubo_ros2_driver --verbose
bash scripts/configure_aubo_realtime.sh
```

使用普通安装方式以匹配现有install，避免混用--symlink-install的既有文件冲突。策略测试覆盖故障锁存/显式复位、2/3共享边界、-5边界、交替异常不可续期和按时间而非次数计算预算。另离线展开校准版ES3 Xacro，检查6轴限位可解析为数值。

## 实时权限

本机检查到RT Tegra内核，但当前登录会话rtprio为0。新脚本默认只检查；需要管理员实际应用时执行：

```bash
bash ~/workspace/scripts/configure_aubo_realtime.sh --apply --user aubo
```

完整退出并重新登录，重新执行检查并启动驱动。旧进程不会获得新权限。未自动应用系统配置。

## 真机复测与恢复

沿用原启动参数。首先检查FIFO警告是否消失、是否只出现一次模式进入；观察新增`servoJoint result/rpc_ms/attempt_gap_ms`。再以原轨迹比较执行时长、动态跟踪误差及终点误差；记录队列满/忙、RTDE解析错误和故障事件。不要增大t或放宽JTC容差来掩盖断流。

故障后先检查机器人端模式/安全状态及ROS硬件生命周期：

```bash
ros2 control list_hardware_components
ros2 control list_controllers
```

停止当前轨迹、确认机器人静止。根据实际状态显式恢复硬件（必要时先configure再activate），随后重新激活需要的控制器并提交新轨迹；或停止驱动后重新启动。on_error成功后硬件可能退回unconfigured，不能只重复activate。激活会清除残留柜端模式并要求新鲜静止反馈，正常反馈恢复不会自动清故障。

## 未解决的边界

- SDK仍为同步二进制库，1s请求超时没有缩短；150ms模式期限不等于可取消正在进行的RPC。控制循环的5ms硬期限并无保证。新机制能拒绝长尾之后继续旧会话，不能撤回已在柜端执行的命令。
- 真正有IO/配置/拖动切换命令时，仍在write内同步执行；停止模式仍有等待静止和确认循环。需结合SDK线程安全以及ROS导出状态的所有权另做独立通信工作线程改造，不能直接把共享double和SDK客户端移到后台。
- RTDE拆包和降载不是SDK解帧修复。SOF混入JSON尚无已验证补丁，仍需记录SDK/柜端版本、双端抓包和SDK厂商分析。扩展状态刷新率变为50Hz，依赖200Hz TCP/力反馈的上层需重新评估。
- 本次没有连接机器人，没有执行真机轨迹或证明200/400ms长尾已根治。

## 本轮验证结果

- aubo_description、aubo_ros2_driver 普通安装构建通过；保留原有CMake/ROS API弃用警告。
- 新增9项策略gtest通过；colcon test-result统计10项（含测试入口），0 errors / 0 failures / 0 skipped。
- 校准版ES3 Xacro离线展开，六轴command position min/max均可转为数值。
- git diff --check、实时脚本bash -n及默认只读检查通过。
- 没有执行--apply、启动真机驱动、发送运动指令或抓包；真机性能改善尚待复测。
