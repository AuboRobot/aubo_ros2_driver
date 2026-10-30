#!/usr/bin/env bash
# 受限的 setcap 包装器：只允许为本工作区 build/ 下的可执行文件设置
# 控制环所需的实时 capability。
# x86 的 sudoers 只免密这一条命令。setcap 前同步本工作区的 typesupport lib
# 到 ld.so.conf，这样 colcon 不需要再开 sudo -n cp/ldconfig。
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

# 路径只从已校验的二进制推导，不接受调用者传入的目录参数。
sync_ld_conf() {
  local ws_root="${real_bin%%/build/*}"
  if [[ "${ws_root}" == "${real_bin}" || ! -d "${ws_root}/install" ]]; then
    echo "WARN: 无法从二进制定位 install 前缀，跳过 ld.so.conf" >&2
    return 0
  fi
  local install_prefix="${ws_root}/install"
  local ros_lib
  ros_lib="$(readelf -d "${real_bin}" 2>/dev/null | sed -n 's#.*\[ \?\(/opt/ros/[^]]*/lib\)\].*#\1#p' | head -n 1 || true)"
  if [[ -z "${ros_lib}" || ! -d "${ros_lib}" ]]; then
    ros_lib="/opt/ros/${ROS_DISTRO:-humble}/lib"
  fi
  local multiarch
  multiarch="$(dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null || true)"
  if [[ -z "${multiarch}" ]]; then
    multiarch="$(uname -m)-linux-gnu"
  fi

  local -a needed=()
  [[ -d "${ros_lib}" ]] && needed+=("${ros_lib}")
  [[ -d "${ros_lib}/${multiarch}" ]] && needed+=("${ros_lib}/${multiarch}")
  if [[ -d "${install_prefix}/aubo_ros2_driver/lib" ]]; then
    needed+=("$(readlink -f "${install_prefix}/aubo_ros2_driver/lib")")
  fi
  local libdir
  for libdir in "${install_prefix}"/*/lib; do
    [[ -d "${libdir}" ]] || continue
    if compgen -G "${libdir}/lib*__rosidl_typesupport_*.so" > /dev/null; then
      needed+=("$(readlink -f "${libdir}")")
    fi
  done

  local conf=/etc/ld.so.conf.d/aubo_ros2_rt.conf
  local -a unique=() missing=()
  local path seen other
  for path in "${needed[@]}"; do
    seen=0
    for other in "${unique[@]+"${unique[@]}"}"; do
      if [[ "${path}" == "${other}" ]]; then
        seen=1
        break
      fi
    done
    [[ "${seen}" -eq 0 ]] && unique+=("${path}")
  done
  for path in "${unique[@]}"; do
    if [[ ! -f "${conf}" ]] || ! grep -qxF "${path}" "${conf}"; then
      missing+=("${path}")
    fi
  done
  [[ "${#missing[@]}" -eq 0 ]] && return 0

  if [[ ! -f "${conf}" ]]; then
    printf '%s\n' "# aubo_ros2_control_node + setcap（AT_SECURE 忽略 LD_LIBRARY_PATH）" > "${conf}"
  elif [[ -n "$(tail -c1 "${conf}" || true)" ]]; then
    printf '\n' >> "${conf}"
  fi
  printf '%s\n' "${missing[@]}" >> "${conf}"
  chmod 644 "${conf}"
  local out filtered
  out="$(ldconfig 2>&1 || true)"
  filtered="$(printf '%s\n' "${out}" | grep -vE '不是符号链接|is not a symbolic link' || true)"
  if [[ -n "${filtered}" ]]; then
    printf '%s\n' "${filtered}" >&2
  fi
  echo "Updated ${conf}:"
  printf '  %s\n' "${missing[@]}"
}

sync_ld_conf
exec "${SETCAP}" "${CAPS}" "${real_bin}"
