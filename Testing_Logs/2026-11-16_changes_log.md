# 测试变更日志 — 2026-11-16

## 概述

本 session 在 `Bug Verification` 下新增 2 个调试/验证脚本；`Generalized/longevity.py` 的「允许系统蓝牙」为临时改动、已按需求还原。按用户要求本 session 只记录日志、不创建 git commit。

## 新增文件

### 1. `Test Cases/Automation/Bug Verification/400 ppg battery/verify_ppg400_battery_loss.py`

- 目的：验证 PPG 400Hz 下，SDK 内部电池查询频率（`init` 的 `powerRefreshInterval`）对 PPG 丢包的影响。
- 4 档测试（每档完整重连，各采集 5 分钟）：

  | 档位 | powerRefreshInterval | 采集时长 |
  |---|---|---|
  | 10 秒查询 | 10000ms | 5 分钟 |
  | 1 秒查询 | 1000ms | 5 分钟 |
  | 500ms 查询 | 500ms | 5 分钟 |
  | 0（不查询，对照组） | 0ms | 5 分钟 |

- 每档流程：scan → connect → Ready → `init(20, interval, 10000)` → 关闭除 PPG 外所有流 → `NTF_PPG=ON` → `PPG_SAMPLE_RATE=400` → 读回确认采样率 → 起流采集 → disconnect。
- 丢包口径：`startSampleIndex` 连续性缺口（样本级）+ `SensorData.lostPackageCount`（SDK 丢包计数）双重统计。
- 设备身份不硬编码，走 `config.TARGET_IDENTITY`（当前 851C）。

### 2. `Test Cases/Automation/Bug Verification/reconnect after close bluetooth/reconnect_after_close_bluetooth.py`

- 目的：验证关闭蓝牙断连 → 重开蓝牙 → 手动重连（不设自动重连）能否恢复数据流。
- 流程：
  1. 连接（`autoReconnect=False`）→ init → 起流，确认有数据
  2. 提示用户【关闭】系统蓝牙 → 等待 10 秒（每秒打印 `deviceState` 观察断开）
  3. 提示用户【开启】系统蓝牙 → 确认后等 2 秒
  4. 清理旧状态（`stopDataNotification` + `disconnect`）
  5. 重新扫描 → connect → init → 起流 → 采集 10 秒数据 → 结束
- 双数据监视器（阶段1 / 阶段4）分别统计，结论按阶段4 是否收到数据判「通过/未通过」。
- 修复：`init` 第三参不再引用不存在的 `config.INIT_TIMEOUT_MS`，改用本地常量 `INIT_TIMEOUT_MS = 10000`。

## 变更说明（未提交 / 无需提交）

- `Generalized/longevity.py`：曾临时放开「允许系统蓝牙（bleak）后端」，后按需求还原为「仅 USB dongle」（净变更 0，git 显示 modified 为 autocrlf 假阳性）。
- `Bug Verification/Disconnect Issue Debug/simulate_oy_app_sequence.py`：起流开关（PPG/EEG/IMU）+ 采样率、电量 <5% 停止、显式 `autoReconnect=False` 等改动，已在上一笔提交 `ab6b7bd cases update, new api to test` 中，无需重复提交。

## 提交状态

- 本 session 未创建 git commit（按用户要求仅记录日志）。
