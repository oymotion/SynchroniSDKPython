# OYWW1000 腕带 SDK 测试用例（总览）

本目录针对 **OYWW1000 腕带**，将《测试计划.md》按模块拆分为多个用例文件。每个用例统一标注：测试目的、流程与逻辑、预期结果、有效性说明、是否可自动化、人工介入。用例 ID 沿用测试计划，便于追溯。

## 设备信息

| 项 | 内容 |
|----|------|
| 设备型号 | OYWW1000（腕带） |
| 广播名前缀 | `OYWW`（兼容 `OY`） |
| 类型归属 | OYM 命名设备 / 传统 OYMotion 设备（非“新 EMG 设备”） |
| SDK 版本 | 0.8.0 |

## 能力假设（连接后用 getDeviceInfo() 确认）

| 能力 | 判定字段（DeviceInfo） | 假设 | 状态 |
|------|------------------------|------|------|
| 肌电 EMG | `EmgChannelCount > 0` | 支持 | 适用 |
| 手势 GEST | 起流后看 `NTF_GEST` 是否有数据 | 支持，与 EMG 互斥 | 待确认 |
| 加速度 ACC | `AccChannelCount > 0` | 支持 | 适用 |
| 陀螺仪 GYRO | `GyroChannelCount > 0` | 支持 | 适用 |
| 欧拉角 Euler | `EulerChannelCount > 0` | 支持 | 适用 |
| 四元数 Quat | `QuatChannelCount > 0` | 支持 | 适用 |
| 聚合 IMU（`NTF_IMU`） | `ImuChannelCount > 0` | 不支持（仅新 EMG） | 待确认 |
| 磁角度 MAG_ANGLE | `MagAngleChannelCount > 0` | 腕带通常无 | 待确认 |
| 阻抗 IMPEDANCE | `ImpeChannelCount > 0` | 腕带通常无 | 待确认 |
| EEG/ECG/PPG/SpO2/BRTH | 对应 `ChannelCount > 0` | 腕带通常无 | 待确认 |

> “待确认”项不省略，执行时以 `getDeviceInfo()` 为准：字段 `>0` 才执行对应用例，否则记录“设备不支持，跳过”。

## 环境与前置条件

- Python 3.10~3.14，`sensor-sdk==0.9.1`（`pip install --upgrade sensor-sdk`）。
- 蓝牙开启（bleak 后端）或 dongle 已绑定（bumble 后端，`checkSetupDongle()` 返回 `OK`）。
- 设备上电、在扫描范围内、广播名以 `OYWW` 开头。
- 每个脚本结束调用 `SensorControllerInstance.terminate()`；Ctrl+C 异常路径也需调用。
- 连接/起流/停流等操作之间留 ≥100ms 间隔。
- 设备 MAC、前缀写入独立 `config.py`（不硬编码）。

## 文件索引

| 文件 | 内容 |
|------|------|
| [01_控制器与扫描.md](./01_控制器与扫描.md) | `CTRL-*`：扫描、requireSensor、后端、版本、终止 |
| [02_连接与状态机.md](./02_连接与状态机.md) | `DEV-SM-*`：连接、断连、重连、状态迁移 |
| [03_数据流.md](./03_数据流.md) | `DATA-*`：起流、样本访问、EMG/GEST/IMU |
| [04_参数.md](./04_参数.md) | `PARAM-*`：setParam/getParam |
| [05_多设备同步.md](./05_多设备同步.md) | `MULTI-*`：同步起流/停流（需 ≥2 台） |
| [06_Bin录制回放解析.md](./06_Bin录制回放解析.md) | `BIN-*`：录制/回放/CSV |
| [07_电量日志调试.md](./07_电量日志调试.md) | `BATT-*` / `LOG-*` |
| [08_人工硬件用例.md](./08_人工硬件用例.md) | `HW-MAN-*`：人工/仪器用例 |
| [09_异步接口.md](./09_异步接口.md) | `ASYNC-*`：async 变体接口 |
| [10_底层接口与边界补充.md](./10_底层接口与边界补充.md) | `MISC-*`：序列化/复用/Invalid/DeviceInfo |

## 执行方式汇总

| 模块 | auto | semi-auto | manual |
|------|:----:|:---------:|:------:|
| 控制器与扫描 | 多数 | 蓝牙开关、dongle 引导 | — |
| 连接与状态机 | 多数 | 断连/断电恢复 | — |
| 数据流 | 多数 | 手势/运动激发 | — |
| 参数 | 多数 | — | — |
| 多设备同步 | 多数 | 后端切换 | — |
| Bin | 多数 | 磁盘不足 | — |
| 电量/日志 | 多数 | 电量变化观察 | — |
| 异步接口 | 多数 | — | — |
| 底层接口与边界 | 多数 | Invalid 触发 | — |
| 人工硬件 | — | — | 全部 |

## 判定标准

- P0 用例 100% 通过；P1 ≥95%；P2 ≥90%。
- 失败保留 SDK 日志与 `.bin` 用于复现。
- 缺设备（如多设备同步需 ≥2 台）时整体“跳过”，不判失败。
