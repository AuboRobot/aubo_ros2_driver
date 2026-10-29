#!/usr/bin/env bash
# 单臂模式切换辅助（对齐 dual aubo_mode_manager 语义，不引入双臂包）
# 用法: switch_mode.sh SERVO|NORMAL|TEACH|UNLOCK
set -euo pipefail
MODE="${1:-}"
case "${MODE}" in
  SERVO|0)
    echo "SERVO: 由 trajectory/servo 控制器接管；确保机器人 Running 后启动 joint_trajectory_controller"
    echo "可先: ros2 service call /robot_manage/startup std_srvs/srv/Trigger {}"
    ;;
  NORMAL|1)
    echo "NORMAL: 停伺服 — 通过 controller_manager 卸载运动控制器；或 dashboard power 保持 Running"
    echo "示例: ros2 control switch_controllers --deactivate joint_trajectory_controller"
    ;;
  TEACH|2)
    echo "TEACH: 单臂请用示教器/ARCS freedrive；本仓库未移植 dual /enable_freedrive 服务"
    exit 2
    ;;
  UNLOCK)
    ros2 service call /robot_manage/unlock_protective_stop std_srvs/srv/Trigger {}
    ;;
  *)
    echo "用法: $0 SERVO|NORMAL|TEACH|UNLOCK" >&2
    exit 1
    ;;
esac
