#!/usr/bin/env bash
set -euo pipefail
# /proc/pid/comm 最长 15 字符，完整名会被截成 aubo_ros2_contr，-x 精确匹配会失败
PID=$(pgrep -f '[a]ubo_ros2_control_node' | head -1 || true)
if [[ -z "${PID}" ]]; then
  echo "aubo_ros2_control_node 未运行"
  exit 1
fi
echo "PID=$PID"
EXE="$(readlink -f "/proc/${PID}/exe" 2>/dev/null || true)"
echo "EXE=${EXE:-unknown}"
if [[ -n "${EXE}" ]]; then
  echo -n "getcap: "
  getcap "${EXE}" || true
  echo -n "mount: "
  findmnt -no TARGET,OPTIONS -T "${EXE}" || true
fi
awk '/^CapEff:/{print}' "/proc/${PID}/status"
ps -o pid,cls,rtprio,cmd -p "$PID"
echo "---- threads ----"
ps -T -p "$PID" -o tid,cls,rtprio,comm
FF=$(ps -T -p "$PID" -o cls= | awk '$1=="FF"{c++} END{print c+0}')
echo "FF_threads=$FF"
LOG=$(ls -t "$HOME"/.ros/log/aubo_ros2_control_node_${PID}_*.log 2>/dev/null | head -1 || true)
if [[ -n "${LOG}" ]]; then
  echo "LOG=$LOG"
  grep -E 'FIFO|RLIMIT|servoJoint t=' "$LOG" | head -20
fi
if [[ "$FF" -gt 0 ]]; then
  echo "RESULT: FIFO 已生效"
  exit 0
fi
echo "RESULT: FIFO 未生效"
exit 2
