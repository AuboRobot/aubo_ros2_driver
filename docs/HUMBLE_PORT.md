# aubo_ros2_driver — Humble port notes

## Source

| Field | Value |
|---|---|
| AGX path | `/home/aubo/workspace/src/aubo_ros2_driver` |
| Sync date | 2026-10-29 (Asia/Shanghai) |
| Base HEAD | `dc4c79f` (`jazzy`, 10 commits ahead of `origin/jazzy`) |
| Dirty snapshot | local branch `agx-jazzy-dirty-sync` @ `10ffe67` |
| Description submodule | base `47fa5e0`; dirty snapshotted as `4fb8dfb` on submodule branch `agx-jazzy-dirty-sync` |
| Upstream | `https://github.com/AuboRobot/aubo_ros2_driver.git` |
| Package version | 2.3.2 |
| SDK | `aubo_sdk-0.27.0-rc.4` (`Linux_x86_64+b7ed04d` on 3060) |

Port work: branch **`humble-port`**. No secrets in this tree. Dual-arm `learn/` not synced.

## Layout on aubo-3060

```text
~/repos/aubo_ros2_driver          # canonical (rsync from AGX, includes .git + submodule)
~/ws_humble/src/aubo_ros2_driver  # symlink → ~/repos/aubo_ros2_driver
```

Repo root is a multi-package colcon tree; symlinking the whole repo into `ws_humble/src/` is correct.

## Humble changes (this port)

1. **`AUBO_SDK_ROOT` override** (`aubo_ros2_driver/CMakeLists.txt`)  
   Prefer pre-extracted SDK; else FetchContent downloads arch-matched zip from `download.aubo-robotics.cn`.
   ```bash
   export AUBO_SDK_ROOT=$HOME/ws/aubo/aubo_sdk-0.27.0-rc.4-Linux_x86_64+b7ed04d
   ```

2. **`realtime_tools::RealtimePublisher`** — Jazzy `try_publish` → Humble `tryPublish`  
   (`aubo_controllers/src/aubo_io_status_controller.cpp`)

3. **`controller_manager` include** — Humble ament include root is  
   `/opt/ros/humble/include/controller_manager`, so use  
   `#include "controller_manager/controller_manager.hpp"`  
   (not the double-nested `controller_manager/controller_manager/...` path).  
   (`aubo_ros2_driver/src/aubo_ros2_control_node.cpp`)

4. **`aubo_description` meshes** — broken relative symlinks under `meshes/aubo_i10HL/visual/link{0..5}.DAE`  
   (pointed at missing `aubo_i10L/.../*.DAE`) retargeted to `aubo_i10`; added matching DAE links under `aubo_i10L` for consistency. Without this, `--symlink-install` fails because CMake `EXISTS` follows the new symlink to a missing target.

5. **`aubo_description/CMakeLists.txt`** — bumped `cmake_minimum_required` 2.8.3 → 3.8 (silence deprecation).

## Build (verified 2026-10-29 Asia/Shanghai)

```bash
source /opt/ros/humble/setup.bash
export AUBO_SDK_ROOT=$HOME/ws/aubo/aubo_sdk-0.27.0-rc.4-Linux_x86_64+b7ed04d
# optional if FetchContent needed:
# export http_proxy=http://127.0.0.1:7897 https_proxy=http://127.0.0.1:7897
cd ~/ws_humble
colcon build --symlink-install
```

All 8 packages: `aubo_msgs`, `aubo_dashboard_msgs`, `aubo_description`, `aubo_controllers`, `aubo_ros2_driver`, `aubo_gazebo`, `aubo_moveit_config`, `ros_joints_plan`.

## Blockers / notes

- Upstream has no `humble` branch; this is a local port from AGX Jazzy dirty tree.
- Gazebo classic packages (`ros-humble-gazebo-ros`, `gazebo-ros2-control`) are available via apt but not required to *build* `aubo_gazebo` (install-only package).
- Do not push AGX credentials; SDK zip is public download / local cache only.


## Dual 对齐（见 DUAL_ALIGN.md）

分支 `dual-align-arcs`：自动上电、防护停止清理、setcap FIFO、remote RViz、`robot_ip` 默认 `127.0.0.1`（ARCS sim-as-real）。
