#!/usr/bin/env bash
# CMake ALL / install / 实机 launch 补回 aubo_ros2_control_node 的 FIFO file capabilities。
# 失败不阻断编译（exit 0）。S100 重启会丢掉 xattr；增量 colcon 若不 relink，
# 仅 POST_BUILD 不会执行，因此必须在每次编译/启动时重打。
#
# 提权分两条路径，互不干扰：
#   x86 PC: NOPASSWD 仅限 /usr/local/sbin/aubo-setcap-rt。wrapper 在 setcap 时
#           同步本工作区 typesupport lib 到 ld.so.conf。不可 sudo -n cp/ldconfig。
#   S100:   sunrise 常有 NOPASSWD: ALL，可 sudo -n setcap 并写入 ld.so.conf。
set -u
BIN="${1:-}"
if [[ -z "${BIN}" || ! -e "${BIN}" ]]; then
  echo "WARN: apply_aubo_rt_caps.sh 缺少二进制参数" >&2
  exit 0
fi
REAL="$(readlink -f "${BIN}")"
SETCAP="/usr/sbin/setcap"
if [[ ! -x "${SETCAP}" ]]; then
  SETCAP="$(command -v setcap || true)"
fi
CAPS="cap_sys_nice,cap_sys_resource+ep"
RT_WRAPPER="/usr/local/sbin/aubo-setcap-rt"

# 通用免密 sudo（S100）。失败时吞掉「需要密码」，避免污染 colcon stderr。
have_passwordless_sudo() {
  [[ "${EUID}" -eq 0 ]] && return 0
  sudo -n true >/dev/null 2>&1
}

run_root() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    sudo -n "$@"
  fi
}

apply_caps() {
  # 1) x86：受限 wrapper（sudoers 只放行这一条）
  if [[ -x "${RT_WRAPPER}" ]]; then
    if [[ "${EUID}" -eq 0 ]]; then
      if "${RT_WRAPPER}" "${REAL}"; then
        return 0
      fi
    elif sudo -n "${RT_WRAPPER}" "${REAL}"; then
      return 0
    fi
    if have_passwordless_sudo; then
      echo "WARN: 受限 wrapper 无法设置 ${REAL}，尝试 sudo setcap" >&2
    fi
  fi
  if [[ -z "${SETCAP}" ]]; then
    echo "WARN: 未找到 setcap" >&2
    return 1
  fi
  # 2) S100 / 已开放免密 sudo：直接 setcap
  if have_passwordless_sudo; then
    run_root "${SETCAP}" "${CAPS}" "${REAL}"
    return $?
  fi
  return 1
}

workspace_lib_from_bin() {
  local real="$1"
  local prefix
  case "${real}" in
    */build/aubo_ros2_driver/*)
      prefix="${real%%/build/aubo_ros2_driver/*}"
      echo "${prefix}/install/aubo_ros2_driver/lib"
      ;;
    */lib/aubo_ros2_driver/*)
      echo "$(cd "$(dirname "${real}")/.." && pwd)"
      ;;
  esac
}

ld_conf_has_path() {
  local conf="$1" path="$2"
  [[ -f "${conf}" && -r "${conf}" ]] || return 1
  grep -qxF "${path}" "${conf}"
}

# mktemp 默认 0600；sudo cp 会把该权限带到 /etc，普通用户随后 grep 报「权限不够」。
make_ld_conf_world_readable() {
  local conf="$1"
  [[ -f "${conf}" && ! -r "${conf}" ]] || return 0
  have_passwordless_sudo || return 1
  run_root chmod 644 "${conf}"
}

# Orbbec 把 libOrbbecSDK.so.2 打成普通文件而非 SONAME 符号链接；ldconfig 仍成功。
run_ldconfig() {
  local out rc=0
  if [[ "${EUID}" -eq 0 ]]; then
    out="$(ldconfig 2>&1)" || rc=$?
  else
    out="$(sudo -n ldconfig 2>&1)" || rc=$?
  fi
  if [[ -n "${out}" ]]; then
    local filtered
    filtered="$(printf '%s\n' "${out}" | grep -vE '不是符号链接|is not a symbolic link' || true)"
    if [[ -n "${filtered}" ]]; then
      printf '%s\n' "${filtered}" >&2
    fi
  fi
  return "${rc}"
}

install_ld_conf() {
  local src="$1" dest="$2"
  if [[ "${EUID}" -eq 0 ]]; then
    install -m 0644 "${src}" "${dest}"
  else
    run_root install -m 0644 "${src}" "${dest}"
  fi
}

ensure_ld_conf() {
  local ros_distro="${ROS_DISTRO:-humble}"
  local ros_lib="/opt/ros/${ros_distro}/lib"
  local multiarch
  multiarch="$(dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null || true)"
  if [[ -z "${multiarch}" ]]; then
    multiarch="$(uname -m)-linux-gnu"
  fi
  local ros_lib_arch="${ros_lib}/${multiarch}"
  local ws_lib
  ws_lib="$(workspace_lib_from_bin "${REAL}")"
  local ld_conf=/etc/ld.so.conf.d/aubo_ros2_rt.conf
  local needed=("${ros_lib}")
  if [[ -d "${ros_lib_arch}" ]]; then
    needed+=("${ros_lib_arch}")
  fi
  if [[ -n "${ws_lib}" && -d "${ws_lib}" ]]; then
    needed+=("${ws_lib}")
    # rmw 按 SONAME dlopen typesupport，不走插件 RUNPATH。AT_SECURE 又忽略
    # LD_LIBRARY_PATH，所以工作区消息包的 lib 必须进 ld.so.cache。
    local install_prefix libdir
    install_prefix="$(cd "${ws_lib}/../.." && pwd)"
    for libdir in "${install_prefix}"/*/lib; do
      [[ -d "${libdir}" ]] || continue
      if compgen -G "${libdir}/lib*__rosidl_typesupport_*.so" > /dev/null; then
        needed+=("$(readlink -f "${libdir}")")
      fi
    done
  fi

  make_ld_conf_world_readable "${ld_conf}" || true

  # 只追加缺失路径，保留 conf 里已有的其他工作区条目。
  local -a missing_paths=()
  local p seen q
  local -a unique_needed=()
  for p in "${needed[@]}"; do
    seen=0
    for q in "${unique_needed[@]+"${unique_needed[@]}"}"; do
      if [[ "${p}" == "${q}" ]]; then
        seen=1
        break
      fi
    done
    [[ "${seen}" -eq 0 ]] && unique_needed+=("${p}")
  done
  for p in "${unique_needed[@]}"; do
    if [[ ! -f "${ld_conf}" ]] || ! ld_conf_has_path "${ld_conf}" "${p}"; then
      missing_paths+=("${p}")
    fi
  done
  if [[ ${#missing_paths[@]} -eq 0 ]]; then
    return 0
  fi

  if ! have_passwordless_sudo; then
    echo "WARN: ${ld_conf} 缺少 ROS/工作区 lib 路径且无免密 sudo；setcap 后若节点找不到 .so，请运行: bash scripts/configure_aubo_realtime.sh" >&2
    printf '  %s\n' "${missing_paths[@]}" >&2
    return 1
  fi

  local tmp rc=0
  tmp="$(mktemp)"
  if [[ -f "${ld_conf}" ]]; then
    cp "${ld_conf}" "${tmp}"
    if [[ -n "$(tail -c1 "${tmp}" || true)" ]]; then
      printf '\n' >> "${tmp}"
    fi
  else
    printf '%s\n' "# aubo_ros2_control_node + setcap（AT_SECURE 忽略 LD_LIBRARY_PATH）" > "${tmp}"
  fi
  printf '%s\n' "${missing_paths[@]}" >> "${tmp}"
  if install_ld_conf "${tmp}" "${ld_conf}" && run_ldconfig; then
    echo "Updated ${ld_conf}:"
    printf '  %s\n' "${missing_paths[@]}"
  else
    echo "WARN: 无法写入 ${ld_conf}；setcap 后可能因 AT_SECURE 找不到 ROS .so" >&2
    rc=1
  fi
  rm -f "${tmp}"
  return "${rc}"
}

if apply_caps; then
  getcap "${REAL}" || true
  ensure_ld_conf || true
  exit 0
fi
echo "WARN: 无法自动 setcap ${REAL}，请运行: bash scripts/configure_aubo_realtime.sh" >&2
exit 0
