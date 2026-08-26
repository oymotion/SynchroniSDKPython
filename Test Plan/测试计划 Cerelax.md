# SynchroniSDKPython 测试计划（Cerelax）

## 1. 文档信息

| 项目 | 内容 |
|------|------|
| 被测对象 | `sensor-sdk`（OYMotion / Synchroni BLE 传感器 Python SDK） |
| 目标设备 | **Cerelax**（2 通道脑电头环；广播名前缀待确认，需实测） |
| 目标版本 | **0.9.6** |
| 测试范围 | SensorController、SensorProfile、数据流（EEG 2 通道为核心，PPG/IMU/Impedance 为重点）、参数（EEG_SAMPLE_RATE 唯一值 250 为核心）、Bin 录制/回放/解析、日志、电量、异步接口、基础数据结构、多设备同步 |
| 文档状态 | 计划（待评审） |
| 前置输入 | [README.md](../README.md)、[examples](../examples)、[Cerelax 手工用例](../Test%20Cases/Manual%20Cases/Cerelax) |

> 说明：本计划为“测试计划文档”，仅定义测试策略与用例设计，不包含自动化脚本实现。用例字段遵循软硬件集成测试模板（正/逆/边界/组合/状态迁移/异常恢复）。
> 本计划由 [OB6000C 测试计划](./测试计划%20OB6000C.md) 派生，差异在于：**Cerelax 为 2 通道脑电头环**（OB6000C 为 32 通道），**新增 PPG / IMU / Impedance 电极接触检测**，**EEG_SAMPLE_RATE 唯一值 250**，不支持 ECG/EMG/GEST/BRTH；IMU 聚合流与四路独立流待运行时确认；IMU/PPG 采样率设置为规划中功能。

---

## 2. 测试范围与对象

### 2.1 被测 SDK 模块

| 模块 | 公开接口（被测点） |
|------|--------------------|
| 控制器 `SensorController` | `isEnable`、`onEnableCallback`、`hasDeviceFoundCallback`、`onDeviceFoundCallback`、`startScan(period_in_ms)`、`scan(period_in_ms)`、`asyncScan`、`stopScan()`、`isScanning`、`requireSensor(device)`、`getSensor(mac)`、`getConnectedSensors()`、`getConnectedDevices()`、`getBLEBackendName()`、`checkSetupDongle()`、`multiStartDataNotification(...)`、`asyncMultiStartDataNotification(...)`、`multiStopDataNotification(...)`、`asyncMultiStopDataNotification(...)`、`getBinFileInfo(path)`、`replayBinFile(...)`、`pauseBinReplay/resumeBinReplay/stopBinReplay`、`parseBinToCsv(...)`、`setDebugEnabled(bool)`、`setLogPath(...)`、`log(msg, level)`、`getVersion()`、`terminate()` |
| 设备 `SensorProfile` | 回调 `onStateChanged/onErrorCallback/onPowerChanged/onDataCallback/onAutoReconnect/onDeviceInfoUpdate`、`autoReconnect`、`isReady`、`connect()/asyncConnect`、`disconnect()/asyncDisconnect`、`deviceState`、`BLEDevice`、`getDeviceInfo()`、`init(...)/asyncInit`、`hasInited`、`startDataNotification()/asyncStartDataNotification`、`stopDataNotification()/asyncStopDataNotification`、`isDataTransfering`、`getBatteryLevel()/asyncGetBatteryLevel`、`setParam/asyncSetParam`、`getParam/asyncGetParam`、`log(msg, level)` |
| 枚举/数据结构 | `DeviceStateEx`（Disconnected/Connecting/Connected/Ready/Disconnecting/Invalid）、`DataType`（16 种，见 2.3）、`DeviceInfo`、`SensorData`、`Sample`、`BLEDevice` |

### 2.2 Cerelax 设备画像（2 通道脑电头环）

| 维度 | 说明 |
|------|------|
| 命名/识别 | 广播名前缀待确认（需实测）；2 通道脑电头环，佩戴式 |
| 核心数据流 | `NTF_EEG`（2 通道脑电 uV）、`NTF_PPG`、`NTF_IMU`、`NTF_IMPEDANCE`（电极接触检测） |
| 核心参数 | `EEG_SAMPLE_RATE`，唯一合法值 `250`（`500`/`1000` 为非法值，实测确认） |
| 采样率 | EEG `250`（唯一）；IMU `50\|100\|200`；PPG `50\|100\|200\|400`（IMU/PPG 为只读标称，无设置接口） |
| IMU 布局 | 聚合流（`NTF_IMU`）或四路独立（`NTF_GFORCE_ACC/GYRO/EULER/QUAT`），待 `getDeviceInfo().ImuChannelCount` 运行时确认 |
| 电极接触 | `NTF_IMPEDANCE` 用于头环佩戴/电极接触检测（Cerelax 数据质量前置） |
| 不支持模态 | ECG/EMG/GEST/BRTH（预计 `ChannelCount==0`） |
| IMU/PPG 采样率设置 | 规划中功能，当前 SDK 无 `IMU_SAMPLE_RATE`/`PPG_SAMPLE_RATE` 设置接口 |
| 远程控制 | 非 NeuCir 设备，`NEUCIR_*` 参数不适用 |

> 关键结论：**EEG 2 通道 + PPG/IMU/Impedance + EEG_SAMPLE_RATE（唯一 250）是 Cerelax 的测试重点**；IMU 布局（聚合/四路）与各模态“支持与否”本身就是测试结果（能力判定），不能预先写死。

### 2.3 数据流类型清单（DataType）

`NTF_ACC`、`NTF_GYRO`、`NTF_EULER_DATA`、`NTF_QUATERNION`、`NTF_GEST`、`NTF_EMG`、`NTF_MAG_ANGLE_DATA`、`NTF_EEG`、`NTF_ECG`、`NTF_IMPEDANCE`、`NTF_IMU`、`NTF_ADS`、`NTF_BRTH`、`NTF_IMPEDANCE_EXT`、`NTF_SPO2`、`NTF_PPG`。

Cerelax 核心关注：`NTF_EEG`（2 通道）、`NTF_PPG`、`NTF_IMU`、`NTF_IMPEDANCE`；能力门控关注：`NTF_ACC`、`NTF_GYRO`、`NTF_EULER_DATA`、`NTF_QUATERNION`、`NTF_SPO2`、`NTF_MAG_ANGLE_DATA`；预计不支持：`NTF_ECG`、`NTF_EMG`、`NTF_GEST`、`NTF_BRTH`、`NTF_ADS`。

### 2.4 数据结构字段/接口

**`DeviceInfo`（`getDeviceInfo()` 返回）字段**

| 字段 | 含义 |
|------|------|
| `EegChannelCount` / `EegSampleRate` | EEG 通道数与采样率（Cerelax 核心：`EegChannelCount==2`、`EegSampleRate==250`） |
| `EegMaxSampleRate` / `EcgMaxSampleRate` / `EmgMaxSampleRate` | 设备能力上报的最大采样率（0 = 未上报） |
| `Ppg*` / `Imu*` / `Impe*` | PPG/IMU/Impedance 的 ChannelCount/SampleRate，Cerelax 重点能力 |
| `Acc*/Gyro*/Euler*/Quat*` | IMU 组成模态 ChannelCount/SampleRate，能力门控依据 |
| `Spo2*` / `MagAngle*` | PPG 相关 / 磁角度，能力门控依据 |
| `Ecg*` / `Emg*` / `Brth*` | 预计 0（Cerelax 不支持） |
| `ConnectionIntervalMs` / `PeripheralLatency` / `SupervisionTimeoutMs` | BLE 链路协商参数（仅 bumble 后端；0/-1/0 = 未知） |

**`SensorData` 接口**

- 元数据：`getDeviceMac()` / `getDeviceName()` / `getDataType()` / `getSampleRate()` / `getChannelCount()` / `getSampleCount()` / `getChannelMask()` / `getLostPackageCount()` / `getStartTimeStamp()` / `getStartTimeSec()` / `getDelay()` / `isDataValid()`。
- 整批：`channelSamples` / `startSampleIndex`（只读）、`clone()`（深拷贝）、`clear()`/`reset()`、`to_flatbuffers()`/`from_flatbuffers()`/`from_flatbuffers_pooled()`。
- 单点访问器（`ci`=通道索引，`si`=样本索引）：`getChannelSample(ci,si)`、`getData(ci,si)`、`getRawData(ci,si)`、`getImpedance(ci,si)`、`getSaturation(ci,si)`、`getSampleIndex(ci,si)`、`getTimeStampInMs(ci,si)`、`getAbsTimeStampInSec(ci,si)`、`isLost(ci,si)`。

**`Sample` 只读字段**：`data` / `rawData` / `impedance` / `saturation` / `sampleIndex` / `channelIndex` / `timeStampInMs` / `absTimeStampInSec` / `isLost`。

**回调交付形式**：`onDataCallback` 每次交付 `list[SensorData]`。

> 底层序列化接口（`to_flatbuffers`/`from_flatbuffers_pooled`）的准确签名待实际编码时确认，计划先列用例不遗漏。

---

## 3. 测试环境与前置条件

### 3.1 硬件环境

| 项目 | 要求 |
|------|------|
| 被测设备 | Cerelax 头环 ≥1 台；多设备同步用例（MULTI-*、ASYNC-FUNC-011/012）需 ≥2 台（可与 OB6000C/OYWW1100 混合） |
| 蓝牙接口 | 原生 BLE（bleak 后端）或 USB BLE dongle（bumble 后端，如 `10d7:b012`/`33fa:0012`） |
| 测试主机 | Windows 10/11（本环境）、Linux、macOS（跨平台用例） |
| 电极/佩戴 | Cerelax 为接触式 EEG 头环，测试需佩戴（电极接触头皮）以获得有效数据；Impedance 用例需人工松脱/恢复电极 |

### 3.2 软件环境

- Python 3.10 ~ 3.14
- `pip install --upgrade sensor-sdk`（依赖 `bleak`、`bumble`、`libusb1`、`numpy` 等）
- 示例依赖：`scipy`、`matplotlib`、`PyQt5`（可选）

### 3.3 通用前置条件

- 蓝牙已开启，或 dongle 已正确绑定驱动（`checkSetupDongle()` 返回 `OK`）。
- 设备已上电、在扫描范围内、广播名符合 Cerelax 前缀。
- 每次测试结束调用 `SensorControllerInstance.terminate()`。
- 起流/停流/连接类操作之间间隔 ≥100ms。
- 多设备（异步多设备用例）需拔掉 USB BLE dongle 以规避“dongle 半启用状态破坏 bleak 多设备连接”的问题（见 §8）。

---

## 4. 测试策略

### 4.1 测试维度（12 维度）

| 维度 | 覆盖重点 | 优先级 | 执行方式 |
|------|----------|--------|----------|
| 功能 | 全部公开 API，按正/逆/边界/组合/状态迁移设计 | P0 | 自动 |
| 性能 | 采样率精度、同步对齐、长时间稳定性 | P1 | 自动 |
| 安全性 | 参数边界、异常恢复、急停/断连保护 | P0 | 自动+人工 |
| 健壮性 | 并发、断连重连、不完整帧容错、资源泄漏 | P1 | 自动 |
| 通信专项 | dongle 后端选择、MTU、帧分片 | P0 | 自动 |
| 兼容性 | 跨平台、Python 版本、跨型号混合同步 | P2 | 自动 |
| 系统维护 | `terminate()`、日志开关、bin 导出、`checkSetupDongle` | P1-P2 | 自动 |
| 不适用 | 不可逆/需密钥操作 | — | 人工 |
| 信号完整性 | 同步对齐（SDK 软件用例）；物理波形归硬件团队 | P1 | 人工+仪器 |
| 电源 | 设备供电、上掉电（归硬件团队） | P1 | 人工 |
| 环境可靠性 | 温湿度运行（归硬件团队） | P2 | 人工 |
| 老化 | 连续运行 24h（SDK 长稳见 `DATA-PERF-002`） | P2 | 半自动 |

### 4.2 用例设计方法（强制）

每个数值参数用**边界值 6 点法**；非法输入用**等价类无效值 + 错误推测**；多参数用**判定表**；设备生命周期用**状态迁移**；断连/断电用**异常恢复**。涉及数值型参数（`init(packageSampleCount, powerRefreshInterval)`、`startScan(period_in_ms)`、`EEG_SAMPLE_RATE`）必须覆盖 6 点边界；集合型接口（`multiStart/multiStop`、`getSensor/requireSensor`、bin 文件路径）必须覆盖空/None 入参。

### 4.3 执行方式划分

- **auto**：无需人工/仪器，脚本可判定。
- **semi-auto**：需人工触发物理动作（如拔 dongle、移动设备、断电、佩戴头环），脚本采集与判定。
- **manual**：需仪器或不可逆操作，人工执行并记录。

---

## 5. 测试用例设计

> 下列 case_id 与 [Cerelax 手工用例](../Test%20Cases/Manual%20Cases/Cerelax) 一一对应。能力门控用例（标注“判定后执行/跳过”）在设备不支持时记录“不适用/不支持”，**不算失败**；标注“规划中/待实现”的用例（IMU/PPG 采样率设置）在 SDK 未提供接口前记为“待实现”，不判失败。

### 5.1 SensorController（控制器与扫描）

| case_id | 标题 | 优先级 | 类型 | 执行方式 |
|---------|------|--------|------|----------|
| CTRL-FUNC-001 | 蓝牙未开启时 `isEnable==False`，`startScan` 被拒绝 | P0 | 逆向 | semi-auto |
| CTRL-FUNC-002 | 蓝牙开启时 `isEnable==True`，`onEnableCallback` 触发 | P0 | 正向 | semi-auto |
| CTRL-FUNC-003 | `startScan(period_in_ms)` 成功返回 True，`isScanning` 翻转 | P0 | 正向 | auto |
| CTRL-FUNC-004 | `scan`/`asyncScan` 返回 `list[BLEDevice]`，含 Cerelax（Name/Address/RSSI 非空） | P0 | 正向 | auto |
| CTRL-FUNC-005 | `onDeviceFoundCallback` 每周期触发一次，不重复推送已连接设备 | P0 | 正向 | auto |
| CTRL-FUNC-006 | `stopScan()` 后 `isScanning==False`，回调停止 | P0 | 正向 | auto |
| CTRL-FUNC-007 | `requireSensor(有效 BLEDevice)` 返回 SensorProfile；无效/None 返回 None | P0 | 正/逆 | auto |
| CTRL-FUNC-008 | `getSensor(已创建 mac)` 返回 profile；未创建/错误 mac 返回 None | P1 | 正/逆 | auto |
| CTRL-FUNC-009 | `getConnectedSensors()`/`getConnectedDevices()` 返回当前已连接集合 | P0 | 正向 | auto |
| CTRL-FUNC-010 | `getBLEBackendName()` 返回 `bumble` 或 `bleak`，与 `SENSOR_SDK_BLE_BACKEND` 一致 | P1 | 正向 | auto |
| CTRL-FUNC-011 | `checkSetupDongle()` 引导并返回 `OK: N` 或 `Error: ...` | P1 | 正向 | semi-auto |
| CTRL-FUNC-012 | `getVersion()` 返回非空版本号 | P2 | 正向 | auto |
| CTRL-FUNC-013 | `terminate()` 正常释放，重复调用不抛异常 | P0 | 异常恢复 | auto |
| CTRL-BND-001 | `startScan(period_in_ms)` 6 点边界：`0`、`1`、`MAX-1`、`MAX`、`-1`、`MAX+1` | P1 | 边界 | auto |
| CTRL-ROB-001 | 扫描期间重复 `startScan`/反复 `stopScan`，无死锁无异常 | P1 | 异常恢复 | auto |
| CTRL-ROB-002 | `startScan`/`stopScan`/`scan` 重复调用幂等，无重复扫描线程 | P1 | 异常恢复 | auto |
| CTRL-BND-002 | `getSensor`/`requireSensor` 空串/None/非法对象返回 None 且不崩溃 | P1 | 边界 | auto |

### 5.2 设备连接 / 状态机

| case_id | 标题 | 优先级 | 类型 | 执行方式 |
|---------|------|--------|------|----------|
| DEV-SM-001 | `connect()` 成功，状态迁移 `Disconnected→Connecting→Connected→Ready` | P0 | 状态迁移 | auto |
| DEV-SM-002 | `connect()` 失败返回 False，`onErrorCallback` 收到原因 | P0 | 异常恢复 | semi-auto |
| DEV-SM-003 | 扫描到非 Cerelax 前缀设备时不误连接 | P1 | 逆向 | auto |
| DEV-SM-004 | `disconnect()` 成功，状态迁移 `Ready→Disconnecting→Disconnected` | P0 | 状态迁移 | auto |
| DEV-SM-005 | 起流中调用 `disconnect()` 自动先 `stopDataNotification` 再断开 | P0 | 正向 | auto |
| DEV-SM-006 | `Ready` 前调 `init`/`setParam`/`startDataNotification` 失败且不崩溃 | P0 | 状态迁移 | auto |
| DEV-SM-007 | `BLEDevice` 的 Name/Address/RSSI 与扫描结果一致 | P1 | 正向 | auto |
| DEV-SM-008 | `autoReconnect=True`（默认）异常断链后自动 `connect→init→恢复 setParam→startDataNotification` | P0 | 异常恢复 | semi-auto |
| DEV-SM-009 | `autoReconnect=False` 异常断链后不自动重连 | P1 | 异常恢复 | semi-auto |
| DEV-SM-010 | `onAutoReconnect(restore=True)` 自定义恢复路径生效，返回 True 跳过默认流程 | P1 | 异常恢复 | semi-auto |
| DEV-SM-011 | 多台设备同时 `connect()`，互不干扰（并发） | P0 | 组合 | auto |
| DEV-SM-012 | 目标设备突然断电/超范围：`onStateChanged(Disconnected)` 与 `onErrorCallback` 触发 | P0 | 异常恢复 | semi-auto |
| DEV-SM-013 | `isReady` 属性等于 `deviceState == DeviceStateEx.Ready` | P1 | 正向 | auto |
| DEV-SM-014 | `onDeviceInfoUpdate` 在链路参数/采样率变更时触发，`DeviceInfo` 就地更新 | P1 | 正向 | auto |
| DEV-SM-015 | `connect` 重复调用幂等，不产生重复连接 | P1 | 异常恢复 | auto |
| DEV-SM-016 | `disconnect` 重复调用幂等，最终 `Disconnected` | P1 | 异常恢复 | auto |

### 5.3 数据流（EEG 2 通道 + PPG/IMU/Impedance 为核心）

| case_id | 标题 | 优先级 | 类型 | 执行方式 |
|---------|------|--------|------|----------|
| DATA-FUNC-001 | `init(packageSampleCount, powerRefreshInterval)` 成功返回 True，`hasInited==True` | P0 | 正向 | auto |
| DATA-FUNC-002 | 未 `connect`/`Ready` 前调用 `init`/`startDataNotification` 失败 | P0 | 逆向 | auto |
| DATA-FUNC-003 | `startDataNotification()` 成功后 `isDataTransfering==True`，`onDataCallback` 按批触发 | P0 | 正向 | auto |
| DATA-FUNC-004 | `stopDataNotification()` 成功后 `isDataTransfering==False`，回调停止 | P0 | 正向 | auto |
| DATA-FUNC-005 | 每个 `ChannelCount>0` 的 DataType（EEG/PPG/IMU/Impedance 为核心；ACC/GYRO/Euler/Quat/SpO2/MagAngle 按能力）均能收到对应 `SensorData`，`getDataType()` 匹配 | P0 | 正向 | auto（能力判定） |
| DATA-FUNC-006 | `SensorData` 元数据合法；**EEG 2 通道**：`getChannelCount()==2`、`getChannelMask` 与通道布局一致 | P0 | 正向 | auto |
| DATA-FUNC-007 | `channelSamples`/单点访问器读取一致，`sampleIndex` 单调递增，逐通道遍历无越界 | P0 | 正向 | auto |
| DATA-FUNC-008 | `sample.isLost` 置位样本正确标记，`getLostPackageCount` 与丢包数一致 | P1 | 异常恢复 | auto |
| DATA-FUNC-009 | 每批样本数 ≈ `packageSampleCount`（允许边界批不足） | P1 | 正向 | auto |
| DATA-FUNC-010 | `NTF_IMU` 聚合批（聚合流 or 四路独立 `NTF_GFORCE_*`，运行时判定） | P1 | 正向 | auto（能力判定） |
| DATA-FUNC-011 | `EEG_SAMPLE_RATE` 变更后 data rate 变化（**唯一值 250**）：设置后起流统计实际采样率 | P0 | 正向 | auto |
| DATA-FUNC-012 | `SensorData.clone()` 返回深拷贝，修改副本不影响原对象 | P1 | 正向 | auto |
| DATA-FUNC-013 | `Sample` 新增字段与单点访问器一致 | P1 | 正向 | auto |
| DATA-FUNC-014 | IMU/PPG 采样率变更后 data rate 变化（**规划中**，依赖 IMU/PPG 采样率设置接口） | P1 | 正向 | auto（待实现） |
| DATA-FUNC-015 | EEG 数据真实性：佩戴后样本非全零、方差非零、幅值合理（区分"不支持"与"未佩戴"） | P1 | 正向 | semi-auto |
| DATA-BND-001 | `init(packageSampleCount)` 6 点边界：`1`、`2`、`MAX-1`、`MAX`、`0`、`MAX+1` | P1 | 边界 | auto |
| DATA-BND-002 | `init(_, powerRefreshInterval)` 6 点边界（0、1、上限邻域、上限、负、超上限） | P1 | 边界 | auto |
| DATA-PERF-001 | 实测采样率与标称采样率偏差 ≤ 容差（EEG 250；PPG/IMU 以只读字段为标称参考） | P1 | 性能 | auto |
| DATA-PERF-002 | 连续起流 ≥ 30min 无内存/句柄泄漏、无掉速 | P1 | 性能 | semi-auto |
| DATA-ROB-001 | `init` 重复调用幂等，`hasInited` 保持 True | P1 | 异常恢复 | auto |
| DATA-ROB-002 | `startDataNotification`/`stopDataNotification` 重复调用幂等 | P1 | 异常恢复 | auto |

### 5.4 setParam / getParam（EEG_SAMPLE_RATE 唯一值 250 为核心）

| case_id | 标题 | 优先级 | 类型 | 执行方式 |
|---------|------|--------|------|----------|
| PARAM-FUNC-001 | 每个 `NTF_*` 键 `setParam("ON"/"OFF")` 返回 `OK`，`getParam("NTF")` 状态一致（能力门控） | P0 | 正向 | auto |
| PARAM-FUNC-002 | 每个 `FILTER_*` 键 `setParam("ON"/"OFF")` 返回 `OK`，`getParam("FILTER")` 一致 | P0 | 正向 | auto |
| PARAM-FUNC-003 | `NTF_IMU` 联动四个 `NTF_GFORCE_*`（聚合流 vs 四路独立，运行时判定） | P1 | 组合 | auto（能力判定） |
| PARAM-FUNC-004 | `setParam("EEG_SAMPLE_RATE", "250")` 成功；逆向 `setParam("EEG_SAMPLE_RATE", "500")` 返回 `Error`（**含逆向：500 非法**） | P0 | 正/逆 | auto |
| PARAM-FUNC-005 | `setParam("EEG_SAMPLE_RATE")` 传列表外值/空串返回 `Error` | P0 | 逆向 | auto |
| PARAM-FUNC-006 | `getParam("EEG_SAMPLE_RATE")` 返回当前采样率；`getParam("EEG_SAMPLE_RATE_LIST")` 返回可选值列表（如 `"250"`） | P1 | 正向 | auto |
| PARAM-FUNC-007 | 起流中修改 `NTF_*`/`FILTER_*`/`EEG_SAMPLE_RATE` 自动停流重起并生效 | P0 | 状态迁移 | auto |
| PARAM-FUNC-008 | `setParam` 传未知 key 返回以 `Error` 开头 | P1 | 逆向 | auto |
| PARAM-FUNC-009 | `NEUCIR_*`（Cerelax 非 NeuCir 设备 → 记录“不适用”） | P1 | 正向 | manual |
| PARAM-FUNC-010 | `DEBUG_BLE_DATA_PATH`/`DEBUG_LOG_PATH` 设置/读取/关闭 | P1 | 正向 | auto |
| PARAM-FUNC-011 | IMU/PPG 采样率只读字段合法（`ImuSampleRate`/`PpgSampleRate` 为合法正值，实测数据率相符） | P1 | 正向 | auto |
| PARAM-FUNC-012 | `IMU_SAMPLE_RATE` 设置（**规划中**，当前 SDK 无此接口） | P1 | 正向 | auto（待实现） |
| PARAM-FUNC-013 | `PPG_SAMPLE_RATE` 设置（**规划中**，当前 SDK 无此接口） | P1 | 正向 | auto（待实现） |
| PARAM-BND-001 | `setParam` 各 key 的非法 value（空串、`ON/OFF` 以外、大小写、None） | P1 | 边界 | auto |
| PARAM-BND-002 | `setParam`/`getParam` 空串/None key 返回 `Error` 且不崩溃 | P1 | 边界 | auto |
| PARAM-ROB-001 | 同一 key 重复 `setParam` 幂等 | P1 | 异常恢复 | auto |

### 5.5 多设备同步（需 ≥2 台）

| case_id | 标题 | 优先级 | 类型 | 执行方式 |
|---------|------|--------|------|----------|
| MULTI-FUNC-001 | 两台同时起流，返回 `{mac: bool}` 皆为 True | P0 | 正向 | auto |
| MULTI-FUNC-002 | 两台同时停流 | P0 | 正向 | auto |
| MULTI-FUNC-003 | 某台未 Ready 不影响其余起流 | P1 | 异常恢复 | auto |
| MULTI-FUNC-004 | 首包时差 ≤ `maxDelayDispersionMs`（默认 5ms，超差自动停流重试） | P0 | 性能 | auto |
| MULTI-FUNC-005 | `maxDelayDispersionMs=-1` 跳过时差校验 | P1 | 正向 | auto |
| MULTI-FUNC-006 | 混合型号（Cerelax + OB6000C/OYWW1100）放宽参数（`timeout=60, maxDelayDispersionMs=-1, maxAttempts=5`） | P1 | 组合 | auto |
| MULTI-FUNC-007 | 已起流设备再次 `multiStart`（restart 语义：先停后同刻起流） | P1 | 状态迁移 | auto |
| MULTI-FUNC-008 | bumble（dongle）与 bleak（原生）两后端各测一次同步起流 | P1 | 正向 | semi-auto |
| MULTI-FUNC-009 | 跨设备 `startTimeStamp` 散布与 `getDelay()` 符合预期 | P1 | 性能 | auto |
| MULTI-BND-001 | `multiStart`/`multiStop` 空集合/None/单台/非法参数（`timeout=0/-1`、`maxDelayDispersionMs` 域外值、`maxAttempts=0/-1`） | P1 | 边界 | auto |
| MULTI-ROB-001 | `multiStop` 重复调用幂等 | P1 | 异常恢复 | auto |

### 5.6 Bin 录制 / 回放 / 解析

| case_id | 标题 | 优先级 | 类型 | 执行方式 |
|---------|------|--------|------|----------|
| BIN-FUNC-001 | 成功连接后自动生成 `.bin`（`{DeviceName}_{MAC}_{时间戳}.bin`） | P0 | 正向 | auto |
| BIN-FUNC-002 | `getBinFileInfo(有效 bin)` 返回含 `device_mac/device_name/chip_type/replay_duration` 的 dict | P0 | 正向 | auto |
| BIN-FUNC-003 | `getBinFileInfo(不存在/无 config)` 返回 None | P0 | 逆向 | auto |
| BIN-FUNC-004 | `replayBinFile(path, sensor, realtime=True)` 按原始节奏回放，`onDataCallback` 收到数据 | P0 | 正向 | auto |
| BIN-FUNC-005 | `replayBinFile(realtime=False)` 全速回放 | P1 | 正向 | auto |
| BIN-FUNC-006 | `pauseBinReplay`/`resumeBinReplay`/`stopBinReplay` 返回 `OK`，暂停/恢复/中止生效 | P1 | 状态迁移 | auto |
| BIN-FUNC-007 | 目标 sensor 正在实时起流时拒绝回放 | P1 | 逆向 | auto |
| BIN-FUNC-008 | `parseBinToCsv` 生成 CSV，含 `raw/cmd_send/cmd_recv/event/parsed` 行 | P1 | 正向 | auto |
| BIN-FUNC-009 | 回放还原的 `startTimeStamp/delay` 与实收值一致 | P1 | 正向 | auto |
| BIN-FUNC-010 | bin 逐帧分析：各 DataType package counts 与 live 交叉验证（区分固件未发 vs SDK 未解析） | P0 | 正向 | auto |
| BIN-BND-001 | 默认（未设 `DEBUG_BLE_DATA_PATH`）时不持久落盘，临时文件断开后删除 | P1 | 逆向 | auto |
| BIN-ROB-001 | 磁盘不足（或只读模拟写盘失败）时跳过/停止录制且不影响实时流 | P2 | 异常恢复 | auto |
| BIN-BND-002 | `replayBinFile`/`parseBinToCsv`/`getBinFileInfo` 空串/None/无效路径返回 None/False 且不崩溃 | P1 | 边界 | auto |
| BIN-ROB-002 | 回放控制重复调用幂等 | P1 | 异常恢复 | auto |

### 5.7 电量 / 日志 / 调试

| case_id | 标题 | 优先级 | 类型 | 执行方式 |
|---------|------|--------|------|----------|
| BATT-FUNC-001 | `getBatteryLevel()` 返回 0~100；无有效读数返回 -1 | P0 | 正向 | auto |
| BATT-FUNC-002 | `onPowerChanged` 上报 0~100，±2% 滞回，不报 -1 | P1 | 正向 | semi-auto |
| BATT-FUNC-003 | `powerRefreshInterval` 决定 `onPowerChanged` 周期 | P1 | 性能 | auto |
| BATT-FUNC-004 | 起流期间 `onPowerChanged` 按周期持续回调（无静默） | P1 | 正向 | auto |
| LOG-FUNC-001 | `setDebugEnabled(True)` 创建 controller log；`False` 关闭 | P1 | 正向 | auto |
| LOG-FUNC-002 | `setLogPath(True, dir)` 创建目录；指向已存在文件被拒绝 | P1 | 正/逆 | auto |
| LOG-FUNC-003 | `DEBUG_LOG_PATH=True` 生成每设备 profile log；`getParam("DEBUG_LOG_PATH")` 返回路径 | P1 | 正向 | auto |
| LOG-FUNC-004 | 早期 scan/connect 日志不丢失（buffer 回放） | P2 | 正向 | auto |
| LOG-FUNC-005 | `SensorController.log(msg, level)`/`SensorProfile.log(msg, level)` 写入应用日志（level `D/I/W/E`），带 `[App]` 标记 | P1 | 正向 | auto |
| LOG-BND-001 | `setLogPath`/`log` 空串/None/非法 level 入参被拒绝或忽略且不崩溃 | P1 | 边界 | auto |
| LOG-ROB-001 | `setDebugEnabled`/`setLogPath` 重复调用幂等 | P1 | 异常恢复 | auto |

### 5.8 异步接口（ASYNC）

| case_id | 标题 | 优先级 | 类型 | 执行方式 |
|---------|------|--------|------|----------|
| ASYNC-FUNC-001 | `asyncScan` 返回设备列表，与同步 `scan` 一致 | P0 | 正向 | auto |
| ASYNC-FUNC-002 | `asyncConnect` 成功/失败 | P0 | 正/逆 | auto |
| ASYNC-FUNC-003 | `asyncDisconnect` 成功 | P0 | 正向 | auto |
| ASYNC-FUNC-004 | `asyncInit` 成功/失败 | P0 | 正/逆 | auto |
| ASYNC-FUNC-005 | `asyncSetParam` 设置成功返回 `OK`，同步 `getParam` 可验证 | P0 | 正向 | auto |
| ASYNC-FUNC-006 | `asyncGetParam` 与同步 `getParam` 一致 | P0 | 正向 | auto |
| ASYNC-FUNC-007 | `asyncGetBatteryLevel` 与同步 `getBatteryLevel` 一致 | P1 | 正向 | auto |
| ASYNC-FUNC-008 | `asyncStartDataNotification` 起流成功 | P0 | 正向 | auto |
| ASYNC-FUNC-009 | `asyncStopDataNotification` 停流成功，回调停止 | P0 | 正向 | auto |
| ASYNC-FUNC-010 | async/sync 混用行为明确（asyncio 内同步接口返回 None，纯 async/sync 各自正确） | P0 | 状态迁移 | auto |
| ASYNC-FUNC-011 | `asyncMultiStartDataNotification` 同时起流多台，返回 `{mac: bool}` 皆为 True | P1 | 组合 | auto |
| ASYNC-FUNC-012 | `asyncMultiStopDataNotification` 同时停流多台，返回 `{mac: bool}` 皆为 True | P1 | 组合 | auto |

> 注意：ASYNC 全部在 `asyncio.run()` 协程内执行，**必须使用异步 BLE API**（`asyncConnect/asyncInit/asyncDisconnect/asyncMultiStart/asyncMultiStop`）；在运行中的事件循环里调用同步 `connect()/init()/disconnect()` 会返回 `None`（事件循环冲突）。

### 5.9 基础接口与边界（MISC）

| case_id | 标题 | 优先级 | 类型 | 执行方式 |
|---------|------|--------|------|----------|
| MISC-FUNC-001 | `hasDeviceFoundCallback` 查询 | P2 | 正向 | auto |
| MISC-FUNC-002 | `Sample.reset` 清空字段 | P1 | 正向 | auto |
| MISC-FUNC-003 | `SensorData.clear/reset` 复用 | P1 | 正向 | auto |
| MISC-FUNC-004 | `SensorData.to_flatbuffers/from_flatbuffers` 往返 | P1 | 正向 | auto |
| MISC-FUNC-005 | `SensorData.from_flatbuffers_pooled` 池化 | P1 | 正向 | auto |
| MISC-FUNC-006 | `SensorData.startSampleIndex` 属性存在且值合法 | P1 | 正向 | auto |
| MISC-FUNC-007 | `DeviceStateEx.Invalid` 状态（可能不可达，不可达则记"不适用"） | P2 | 正向 | semi-auto |
| MISC-FUNC-008 | `DeviceInfo` 能力字段完整性（`EegChannelCount==2`、PPG/IMU/Impedance 字段 `>0`、ECG/EMG 为 0） | P1 | 正向 | auto |

### 5.10 人工硬件用例（manual）

| case_id | 标题 | 优先级 | 类型 | 执行方式 |
|---------|------|--------|------|----------|
| HW-MAN-001 | BLE 通信链路 RSSI/连接稳定性人工验证 | P1 | 正向 | manual |
| HW-MAN-002 | USB BLE dongle 驱动绑定与 `checkSetupDongle()` 引导流程 | P1 | 正向 | manual |
| HW-MAN-003 | 设备供电电压/上掉电时序与 SDK 重连恢复 | P1 | 异常恢复 | manual |
| HW-MAN-006 | 头环佩戴与 `NTF_IMPEDANCE` 电极接触检测人工验证 | P1 | 正向 | manual |

> HW-MAN-004（24h 老化）、HW-MAN-005（高低温）超出 SDK 软件测试范围，归硬件/可靠性团队，不在本计划展开。

### 5.11 边界值分析汇总（6 点法）

| 参数 | 范围 | B1~B4（合法） | B5/B6（非法） | 补充 |
|------|------|----------------|---------------|------|
| `startScan(period_in_ms)` | >0 | min、min+1、max-1、max | min-1(0/负)、max+1 | None/字符串 |
| `init(packageSampleCount)` | ≥1 | 1、2、N-1、N | 0、N+1 | None/字符串 |
| `init(powerRefreshInterval)` | ≥0 | 0、1、N-1、N | -1、N+1 | None |
| `EEG_SAMPLE_RATE` | 唯一 `250` | 250 | 500、空串、其他列表外值 | 500 为已知非法值 |
| `getBatteryLevel()` 返回 | 0~100 或 -1 | 0、1、99、100 | — | -1 表示无读数 |
| `multiStart/multiStop` 集合 | ≥2 台 | 2 台、多台 | 空集合、None、单台 | 非法参数见 MULTI-BND-001 |

---

## 6. Cerelax 能力测试矩阵

| 能力 | 核心/门控 | 验证用例 | 预期 |
|------|-----------|----------|------|
| EEG 2 通道数据流 | 核心 | DATA-FUNC-005/006/007/009/013 | `EegChannelCount==2`，逐通道有数据 |
| EEG 采样率（唯一 250） | 核心 | PARAM-FUNC-004/005/006、DATA-FUNC-011 | 250 生效，500 返回 Error |
| PPG 数据流 | 核心 | DATA-FUNC-005、PARAM-FUNC-001 | `PpgChannelCount>0` 有数据 |
| IMU 数据流（聚合 or 四路） | 核心/门控 | DATA-FUNC-005/010、PARAM-FUNC-001/003 | 按 `ImuChannelCount`/四路 `*ChannelCount` 判定 |
| Impedance 电极接触检测 | 核心 | DATA-FUNC-005、HW-MAN-006 | `ImpeChannelCount>0`，松脱电极阻抗异常 |
| IMU/PPG 采样率设置 | 规划中 | PARAM-FUNC-012/013、DATA-FUNC-014 | SDK 未实现前记“待实现” |
| ACC/Gyro/Euler/Quat 独立流 | 门控 | DATA-FUNC-005、PARAM-FUNC-001 | 按 `*ChannelCount>0` 判定 |
| SpO2 / MagAngle | 门控 | DATA-FUNC-005、PARAM-FUNC-001 | 按 `*ChannelCount>0` 判定 |
| ECG / EMG / GEST / BRTH | 不支持 | MISC-FUNC-008、DATA-FUNC-005 | `ChannelCount==0` → 记录“不支持” |
| 连接/状态机/扫描 | 核心 | CTRL-*、DEV-SM-* | 全部通过 |
| Bin 录制/回放/解析 | 核心 | BIN-* | 全部通过 |
| 电量/日志 | 核心 | BATT-*、LOG-* | 全部通过 |
| 异步接口 | 核心 | ASYNC-* | 全部通过 |
| 多设备同步 | 核心 | MULTI-* | 全部通过（≥2 台） |
| 基础数据结构 | 核心 | MISC-* | 全部通过 |

---

## 7. 判定标准与通过率目标

- 单用例通过标准：`expected` 全部命中，且无未捕获异常、无内存/句柄泄漏。
- P0 用例：**100% 通过**；P1：≥95%；P2：≥90%。
- “能力门控”用例（设备不支持而记录“不适用/不支持”）**不纳入失败**，但需在结果中明确标注“能力判定：不支持”。
- “规划中/待实现”用例（IMU/PPG 采样率设置）在 SDK 未提供接口前记为“待实现”，不纳入失败；接口落地后转为正式执行。
- 所有数值参数必须通过 6 点边界值分析；EEG 2 通道、EEG_SAMPLE_RATE（唯一 250）、PPG/IMU/Impedance 必须实测验证。
- 失败用例须保留 SDK 日志（controller/profile log）与 `.bin`，用于复现。

---

## 8. 风险与安全注意事项

1. **连接/断电/断连用例**：执行前确认头环电极不接触他人皮肤；异常注入（拔 dongle、断电）仅允许对测试专用设备执行。
2. **多设备用例（MULTI-*、ASYNC-FUNC-011/012）**：多台设备同时起流时，确保设备固定、佩戴者状态稳定。
3. **dongle 半启用状态**：插入 USB BLE dongle 但后端仍为 `bleak` 时，会破坏 bleak 多设备连接（第二台 `connect()` 返回 False）。**bleak 多设备测试前拔掉 dongle**。
4. **异步接口事件循环**：`asyncio.run()` 协程内必须使用异步 BLE API，同步接口会返回 `None`。
5. **破坏性/不可逆操作**（复位、改 ID、标定等）不纳入自动化，归入人工。
6. **`terminate()` 必须调用**：所有脚本（含异常退出路径）须在 finally 中调用。
7. **测试间留足间隔**：起流/停流/连接类操作之间 ≥100ms。
8. **后端兼容**：`bumble`（dongle）与 `bleak`（原生）两后端各跑一遍通信用例。
9. **回调线程安全**：`onDataCallback` 单线程顺序交付，回调内保持短小或线程安全。
10. **头环佩戴**：EEG/Impedance 用例需真实佩戴以获得有效数据，不佩戴时数据可能为无效/低幅值，需区分“设备不支持”与“未佩戴导致无有效数据”。

---

## 9. 交付物（进入执行阶段后）

- 自动化用例：`Test Cases/Automation/Cerelax`（CTRL/DEV-SM/DATA/PARAM/BIN/BATT/LOG/ASYNC/MISC 各目录，待建立）。
- 手工用例：[`Test Cases/Manual Cases/Cerelax`](../Test%20Cases/Manual%20Cases/Cerelax)。
- 配置：`Test Cases/Automation/config.py`（`TARGET_IDENTITY` 指向 Cerelax identity，待确认）。
- 报告：`Test Reports/`（HTML，含 Bug 汇总与 Redmine 链接）。
