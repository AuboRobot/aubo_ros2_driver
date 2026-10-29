#!/usr/bin/env bash
# 为 aubo_ros2_control_node 设置 CAP_SYS_NICE / CAP_SYS_RESOURCE，使控制环可启用 FIFO RT。
# 注意:
#   1) symlink-install 后 rebuild 会清掉 file capabilities。CMake ALL 目标
#      apply_aubo_rt_caps 在 sudo -n 可用时每次编译都自动补回（不依赖 relink）；
#      否则需重跑本脚本。RDK S100 重启也会丢掉 xattr。
#   2) 若工作空间在 nosuid 挂载点（常见于 /mnt/ssd），setcap 成功但进程 CapEff=0、
#      始终 CLS=TS。本脚本会 remount suid、改 fstab 去掉 nosuid，并用 CapEff 探测确认。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# package scripts/ → prefer colcon ws (~/ws_humble), else repo root
if [[ -n "${COLCON_PREFIX_PATH:-}" ]]; then
  WS_ROOT="$(echo "${COLCON_PREFIX_PATH}" | awk -F: '{print $1}' | xargs -I{} dirname {})"
elif [[ -f "${HOME}/ws_humble/install/setup.bash" ]]; then
  WS_ROOT="${HOME}/ws_humble"
else
  WS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
fi

if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${WS_ROOT}/install/setup.bash"
  set -u
fi

BIN="$(ros2 pkg prefix aubo_ros2_driver 2>/dev/null)/lib/aubo_ros2_driver/aubo_ros2_control_node"
if [[ ! -x "${BIN}" ]]; then
  echo "错误: 未找到可执行文件 ${BIN}"
  echo "请先执行: colcon build --packages-select aubo_ros2_driver --symlink-install"
  exit 1
fi

# symlink-install 时 setcap 必须作用在真实二进制上
REAL_BIN="$(readlink -f "${BIN}")"
if [[ ! -x "${REAL_BIN}" ]]; then
  echo "错误: 无法解析真实二进制 ${REAL_BIN}"
  exit 1
fi

echo "目标: ${BIN}"
echo "真实路径: ${REAL_BIN}"
echo "当前 ulimit -r: $(ulimit -r)"

MNT_OPTS="$(findmnt -no OPTIONS -T "${REAL_BIN}" 2>/dev/null || true)"
MNT_TARGET="$(findmnt -no TARGET -T "${REAL_BIN}" 2>/dev/null || true)"
echo "所在挂载: ${MNT_TARGET:-?}  options=${MNT_OPTS:-?}"

sudo_run() {
  if [[ "${EUID}" -ne 0 ]]; then
    sudo "$@"
  else
    "$@"
  fi
}

capeff_is_zero() {
  local hex="${1:-}"
  hex="${hex//0/}"
  hex="${hex// /}"
  hex="${hex//$'\t'/}"
  [[ -z "${hex}" ]]
}

persist_suid_fstab() {
  local mnt="$1"
  local tmp rc=0
  tmp="$(mktemp)"
  python3 - "$mnt" "$tmp" <<'PY' || rc=$?
import pathlib
import sys

mnt = sys.argv[1]
out_path = pathlib.Path(sys.argv[2])
text = pathlib.Path("/etc/fstab").read_text()
out_lines = []
changed = False
for line in text.splitlines(True):
    stripped = line.split("#", 1)[0]
    parts = stripped.split()
    if len(parts) >= 4 and parts[1] == mnt:
        if "nosuid" not in parts[3].split(","):
            out_lines.append(line)
            continue
        opts = [o for o in parts[3].split(",") if o and o != "nosuid"]
        if "suid" not in opts:
            if opts and opts[0] == "defaults":
                opts.insert(1, "suid")
            else:
                opts.insert(0, "suid")
        new_opts = ",".join(opts)
        if new_opts != parts[3]:
            parts[3] = new_opts
            nl = "\n" if line.endswith("\n") else ""
            line = " ".join(parts) + nl
            changed = True
    out_lines.append(line)
out_path.write_text("".join(out_lines))
sys.exit(0 if changed else 2)
PY
  if [[ "${rc}" -eq 0 ]]; then
    if [[ ! -s "${tmp}" ]] || ! grep -qE "[[:space:]]${mnt}[[:space:]]" "${tmp}"; then
      echo "错误: 生成的 fstab 无效，放弃写入"
      rm -f "${tmp}"
      return 1
    fi
    sudo_run cp -a /etc/fstab "/etc/fstab.bak.aubo-rt"
    sudo_run cp "${tmp}" /etc/fstab
    rm -f "${tmp}"
    return 0
  fi
  rm -f "${tmp}"
  if [[ "${rc}" -eq 2 ]]; then
    return 2
  fi
  return "${rc}"
}

probe_mount_capeff() {
  local probe_parent="$1"
  local dir pid capeff
  dir="$(mktemp -d "${probe_parent}/.aubo_captest.XXXXXX")" || return 1
  if ! cp /bin/sleep "${dir}/sleepcap" ||
     ! chmod 755 "${dir}/sleepcap" ||
     ! sudo_run setcap "cap_sys_nice+ep" "${dir}/sleepcap"; then
    rm -rf -- "${dir}"
    return 1
  fi
  "${dir}/sleepcap" 8 &
  pid=$!
  capeff=""
  for _ in $(seq 1 20); do
    if [[ -r "/proc/${pid}/status" ]]; then
      capeff="$(awk '/^CapEff:/{print $2}' "/proc/${pid}/status")"
      break
    fi
    sleep 0.05
  done
  kill "${pid}" >/dev/null 2>&1 || true
  wait "${pid}" >/dev/null 2>&1 || true
  rm -rf "${dir}"
  echo "${capeff}"
}

if [[ -n "${MNT_TARGET}" ]]; then
  if [[ "${MNT_OPTS}" == *nosuid* ]]; then
    echo "挂载含 nosuid，file capabilities 会被内核忽略。正在 remount suid..."
    sudo_run mount -o remount,suid,dev "${MNT_TARGET}"
  fi
  fstab_rc=0
  persist_suid_fstab "${MNT_TARGET}" || fstab_rc=$?
  if [[ "${fstab_rc}" -eq 0 ]]; then
    echo "已更新 /etc/fstab：去掉 nosuid 并显式 suid（${MNT_TARGET}），备份 /etc/fstab.bak.aubo-rt"
  elif [[ "${fstab_rc}" -eq 2 ]]; then
    echo "/etc/fstab 无需移除 nosuid，保持原配置"
  else
    echo "警告: 未能改写 /etc/fstab（rc=${fstab_rc}）"
  fi
  MNT_OPTS="$(findmnt -no OPTIONS -T "${REAL_BIN}" 2>/dev/null || true)"
  if [[ "${MNT_OPTS}" == *nosuid* ]]; then
    echo "remount/fstab 后仍含 nosuid，中止。"
    echo "  手动: sudo mount -o remount,suid,dev ${MNT_TARGET}"
    exit 2
  fi
  echo "探测挂载是否真正允许 file capabilities..."
  # Probe beside the binary: same filesystem, writable even when mounted at /.
  PROBE_CAPEFF="$(probe_mount_capeff "$(dirname "${REAL_BIN}")")" || {
    echo "错误: 无法创建或配置 capability 探测文件。"
    exit 2
  }
  echo "  CapEff=${PROBE_CAPEFF}"
  if capeff_is_zero "${PROBE_CAPEFF}"; then
    echo "错误: 在 ${MNT_TARGET} 上 setcap 后进程 CapEff 仍为 0（内核忽略 file caps）。"
    echo "  findmnt: ${MNT_OPTS}"
    exit 2
  fi
  echo "挂载 CapEff 探测: OK"
fi

if ! command -v setcap >/dev/null 2>&1; then
  echo "错误: 未安装 setcap (libcap2-bin)"
  exit 1
fi

# file capabilities 会进入 AT_SECURE：忽略 LD_LIBRARY_PATH。
# 将 ROS（含 multiarch）与本包 lib 写入 ld.so.conf，确保间接依赖可加载。
ROS_PREFIX="$(ros2 pkg prefix controller_interface 2>/dev/null || true)"
if [[ -z "${ROS_PREFIX}" ]]; then
  ROS_PREFIX="/opt/ros/${ROS_DISTRO:-humble}"
fi
ROS_LIB="${ROS_PREFIX}/lib"
MULTIARCH="$(dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null || true)"
if [[ -z "${MULTIARCH}" ]]; then
  MULTIARCH="$(uname -m)-linux-gnu"
fi
ROS_LIB_ARCH="${ROS_LIB}/${MULTIARCH}"
WS_LIB="$(readlink -f "$(ros2 pkg prefix aubo_ros2_driver)/lib")"
LD_CONF=/etc/ld.so.conf.d/aubo_ros2_rt.conf
LD_CONF_BODY="# aubo_ros2_control_node + setcap 需要（AT_SECURE 忽略 LD_LIBRARY_PATH）
${ROS_LIB}
"
if [[ -d "${ROS_LIB_ARCH}" ]]; then
  LD_CONF_BODY+="${ROS_LIB_ARCH}
"
fi
LD_CONF_BODY+="${WS_LIB}
"
echo "写入 ${LD_CONF}:"
echo "  ${ROS_LIB}"
[[ -d "${ROS_LIB_ARCH}" ]] && echo "  ${ROS_LIB_ARCH}"
echo "  ${WS_LIB}"
if [[ "${EUID}" -ne 0 ]]; then
  echo "${LD_CONF_BODY}" | sudo tee "${LD_CONF}" >/dev/null
  sudo ldconfig
else
  echo "${LD_CONF_BODY}" | tee "${LD_CONF}" >/dev/null
  ldconfig
fi

echo "需要 sudo 设置 capability..."
# CAP_SYS_NICE: FIFO 调度；CAP_SYS_RESOURCE: 抬高 RLIMIT_RTPRIO（ulimit -r=0 时必需）
sudo_run setcap "cap_sys_nice,cap_sys_resource+ep" "${REAL_BIN}"

echo "Capability (真实二进制):"
getcap "${REAL_BIN}" || true

# 冒烟：无 LD_LIBRARY_PATH 时至少应能解析共享库（会因缺 AMENT 参数异常退出，但不应是 missing .so）
SMOKE_OUT="$(env -i PATH=/usr/bin:/bin HOME="${HOME:-/tmp}" "${REAL_BIN}" 2>&1 || true)"
if echo "${SMOKE_OUT}" | grep -qE "error while loading shared libraries|version .* not found|symbol lookup error"; then
  echo "错误: setcap 后仍无法加载共享库:"
  echo "${SMOKE_OUT}" | head -5
  exit 3
fi
echo "共享库加载检查: OK（AT_SECURE 下可解析 .so）"

echo ""
echo "验证（必须重启 aubo_ros2_control_node 后）:"
echo "  日志应出现: FIFO RT scheduling enabled"
echo "  查控制环线程（主线程可能仍是 TS）:"
echo "    bash ${SCRIPT_DIR}/check_aubo_rt.sh"
echo "  期望至少一条 CLS=FF 且 RTPRIO=50"
echo ""
echo "备选（不依赖 file caps）: pam limits + 重新登录"
echo "  echo '@realtime - rtprio 95' | sudo tee /etc/security/limits.d/99-aubo-realtime.conf"
echo "  sudo groupadd -f realtime && sudo usermod -aG realtime \"\$USER\""
