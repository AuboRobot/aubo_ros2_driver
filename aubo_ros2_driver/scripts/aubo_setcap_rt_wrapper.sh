#!/usr/bin/env bash
# 受限的 setcap 包装器：只允许为本工作区 build/ 下的可执行文件设置
# 控制环所需的实时 capability。
set -euo pipefail

readonly SETCAP="/usr/sbin/setcap"
readonly CAPS="cap_sys_nice,cap_sys_resource+ep"

usage() {
  echo "用法: $0 <workspace>/aubo_ros2_driver/build/<executable>" >&2
  exit 2
}

[[ "$#" -eq 1 ]] || usage
[[ -x "${SETCAP}" ]] || { echo "错误: 未找到 ${SETCAP}" >&2; exit 1; }

input="$1"
[[ "${input}" != -* ]] || usage

real_bin="$(readlink -e -- "${input}" 2>/dev/null || true)"
[[ -n "${real_bin}" && -f "${real_bin}" && -x "${real_bin}" ]] || {
  echo "错误: 目标必须是存在且可执行的普通文件: ${input}" >&2
  exit 1
}

case "${real_bin}" in
  */aubo_ros2_driver/build/*|*/ws_humble/build/*) ;;
  *)
    echo "错误: 目标必须位于 */aubo_ros2_driver/build/ 或 */ws_humble/build/ 下: ${real_bin}" >&2
    exit 1
    ;;
esac

# 通过 sudo 调用时，目标必须仍归发起 sudo 的普通用户所有，避免把 root-owned
# 文件纳入这个通用入口。直接由 root 调用时则要求 root 所有。
expected_uid="${SUDO_UID:-0}"
actual_uid="$(stat -c '%u' -- "${real_bin}")"
if [[ "${actual_uid}" != "${expected_uid}" ]]; then
  echo "错误: 目标文件所有者 UID=${actual_uid}，期望 UID=${expected_uid}" >&2
  exit 1
fi

exec "${SETCAP}" "${CAPS}" "${real_bin}"
