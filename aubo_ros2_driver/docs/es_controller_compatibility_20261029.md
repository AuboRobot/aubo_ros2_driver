# ES ARM / 常规控制柜兼容方案（2026-10-29）

## 适用边界

现场柜端设备树确认Forlinx OK3568-C / RK3568，SDK与服务端接口版本均0.26.0。不能因为ROS运行在AGX或x86就推断远端控制柜类型。ES型号默认选择保守配置，允许显式覆盖；x86柜体未在本轮实测。

## 实施

- controller_profile=auto：aubo_ES开头型号100Hz，其他型号沿用YAML；es=100Hz；legacy=200Hz；custom=YAML。update_rate显式覆盖，25～500Hz。
- servo_period显式传给servoJoint，默认保持0.01；control_period随实际选择频率传给硬件接口，用于RPC预算诊断。校验有限正数、servo_period范围及control_period<5*t。50ms保护改为由5*t计算，默认保护强度不变。
- 在启动前重写临时YAML中controller_manager.ros__parameters.update_rate；实测发现单纯追加Node通配符参数不能覆盖节点专属YAML，现已用实际参数和日志核验。
- 辅助控制器逐个加载/激活，JTC最后启动。任一前置spawner失败会结束启动，不继续启动运动控制器。保留fake硬件分支对辅助控制器的跳过。
- IO/RobotManage/RobotConfig三种状态改为RealtimePublisher.try_publish，后台处理DDS发布，发布忙时跳过旧快照。消息复用并在激活时预热容器。服务名称及字段保持原状。
- 保留Servo返回码时间预算、故障锁存、显式恢复、RTDE新鲜度检查、原增益和轨迹容差；本轮没有改变RTDE频率和配置命令执行路径。

## 测试与证据

生产实现已构建；原9项Servo策略gtest及新增6项profile unittest通过，ctest两个测试入口成功。100/200Hz的ES3校准版Xacro均验证control_period和servo_period数值映射。

真机对照及微幅运动数据见工作区log/servo_es_optimized_20261029、log/servo_legacy200_optimized_20261029及log/ES控制器兼容优化报告.md。初始100Hz试验在旧发布路径下出现启动期340.53ms尝试间隔和330.65ms Update，保护拒绝继续；没有执行轨迹。该故障不足以单独证明阻塞发生于哪个控制器，源码中的并行加载和同步DDS发布均为已移除的实时性风险。

## 启动

```bash
cd ~/workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=aubo_ES3 robot_ip:=192.168.3.100 \
  use_fake_hardware:=false use_calibrated:=true controller_profile:=es
```

显式保留旧路径用controller_profile:=legacy；自定义配置用controller_profile:=custom，必要时update_rate:=100。运行时通过ros2 param get /controller_manager update_rate核验。servo_period=.01为本次已测试组合；不要仅为掩盖断流增大t。

## 尚未保证

同步SDK、真实IO/配置服务仍可能阻塞；RealtimePublisher避免DDS阻塞进入控制线程，但不等于全链路无分配或硬实时认证。100Hz测试不代替满载长时间验证。只有wrist3小幅往返已验证，未评估全工作空间、多轴轨迹、工具环境碰撞、重负载以及x86柜体的200Hz性能。
