# 变更记录

## 未发布（2026-11-13）

### 修复

- `/robot_manage/set_handguide` 不再在命令写入接口后立即返回成功。服务会等到 `robot_manage/get_handguide_enabled` 与请求一致，最多 8 秒。硬件没有确认进入或退出拖动时返回失败。需要重启驱动后才会生效。
