# RehabArm 磁力计数据流用例（MAG-*）

> 本章是 RehabArm 的**磁力计（MAG）专项章（编号 12）**。
> 按设备实际能力执行：`getDeviceInfo()` 对应 `MagChannelCount>0` 才执行；否则记录“不支持，跳过”。

## 设备磁力计能力基线（来自 probe_device.py 实测 + pyqt_demo.py，2026-10-27，设备 1D0E）

| 项 | 值 |
|----|----|
| 能力声明 | `DeviceInfo.MagChannelCount=3`、`MagSampleRate=10` |
| 数据流类型 | `DataType.NTF_MAG`（0x03） |
| 通道数 | 3（X / Y / Z 三轴） |
| 采样率 | 10 Hz |
| 单位 | uT（微特斯拉） |
| 显示标签 | `Magnetometer (MAG)`，通道 `MAG-X (uT)` / `MAG-Y (uT)` / `MAG-Z (uT)` |

> 注意：磁力计是独立流（不并入 IMU 聚合 `ImuChannelCount=13`），
> 与角度(1ch/250Hz)、IMU 四路(各 50Hz) 并存，采样率互不影响。
>
> **流开关注意（2026-11-03 实测，SDK 1.4.3 / 设备 1D0E）**：
> - 磁力计**没有独立 setParam 开关**：`getParam("NTF_MAG")` / `setParam("NTF_MAG", ...)`
>   均返回 `Error: Not supported`。
> - 磁力计随 **`NTF_IMU`** 聚合开关起停：`setParam("NTF_IMU","OFF")` 返回 `OK`，
>   磁力计与 ACC/GYRO/EULER/QUAT 一起停止；`ON` 后一起恢复。
> - 因此「只停磁力计、保留其它流」在当前 SDK/固件上无法实现，相关断言按型号限制记 SKIP。
> - 权威 key 列表取自 `getParam("NTF")`（其中无 MAG）。

---

### MAG-FUNC-001 DeviceInfo 声明磁力计能力
- **优先级**：P0
- **测试目的**：验证设备通过 `DeviceInfo` 正确声明磁力计能力。
- **流程与逻辑**：Ready 后 `init`，`fetchDeviceInfo()` 或 `getDeviceInfo()` 读 `MagChannelCount`、`MagSampleRate`。
- **预期结果**：`MagChannelCount==3` 且 `MagSampleRate==10`。
- **有效性说明**：能力声明是后续磁力计用例的前置判定依据（`>0` 才执行）。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：通过

### MAG-FUNC-002 磁力计流开关 getParam/setParam
- **优先级**：P0
- **测试目的**：验证磁力计流开关参数可读写、可切换。
- **流程与逻辑**：先验证 `getParam("NTF_MAG")`（预期不支持）；再走真实路径 `getParam("NTF_IMU")` 读状态，`setParam("NTF_IMU","ON")` / `setParam("NTF_IMU","OFF")` 各切换一次并回读。
- **预期结果**：`NTF_MAG` 返回 `Not supported`（记 SKIP，型号无独立开关）；`NTF_IMU` 的 `getParam` 返回非 `Error`，`ON`/`OFF` 切换返回 `OK` 且回读一致。
- **有效性说明**：磁力计随 NTF_IMU 聚合开关起停，无独立入口。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：通过

### MAG-FUNC-003 起流后收到 NTF_MAG 且 getDataType 匹配
- **优先级**：P0
- **测试目的**：验证磁力计流真实产出数据，类型正确。
- **流程与逻辑**：开 `NTF_IMU`（磁力计随其起停）→ `startDataNotification()` → 在 `onDataCallback` 统计 `getDataType()`。
- **预期结果**：收到 `DataType.NTF_MAG`（0x03）数据批。
- **有效性说明**：证明磁力计流不是“仅参数接受、无实际数据”。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：通过（2026-11-03 实跑，设备 1D0E：162 样本 / 54 批，getDataType=NTF_MAG）

### MAG-FUNC-004 磁力计 SensorData 元数据合法
- **优先级**：P0
- **测试目的**：验证磁力计批次的元数据接口。
- **流程与逻辑**：对 `NTF_MAG` 批次检查 `getChannelCount()`、`getSampleRate()`、`getSampleCount()`、`isDataValid()`。
- **预期结果**：`getChannelCount()==3`、`getSampleRate()==10`、`getSampleCount()>0`、`isDataValid()==True`。
- **有效性说明**：元数据是上层解析三轴磁强值的依据。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：通过（2026-11-03 实跑，设备 1D0E：channelCount=3、sampleRate=10.0、isDataValid=True）

### MAG-FUNC-005 三轴访问器（X/Y/Z，单位 uT）
- **优先级**：P0
- **测试目的**：验证三轴磁强值的单点访问接口。
- **流程与逻辑**：对 `NTF_MAG` 用 `getData(ci, si)` 访问 ch=0/1/2（对应 X/Y/Z），检查 `getData` 与 `getChannelSample(ci,si).data` 一致、`getSampleIndex` 单调。
- **预期结果**：三通道值均可访问，单位 uT；访问器一致、索引单调。
- **有效性说明**：确认三轴数据可读、单位正确。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：通过（2026-11-03 实跑，设备 1D0E：三轴可访问、访问器一致、sampleIndex 单调）

### MAG-FUNC-006 磁力计与角度/IMU 流共存
- **优先级**：P1
- **测试目的**：验证磁力计与角度、IMU 四路流同时起流、采样率互不干扰。
- **流程与逻辑**：同时开 `NTF_IMU`（含磁力计）与 `NTF_ANGLE`，起流后统计各流 `getSampleRate()`。
- **预期结果**：`NTF_MAG`(10Hz)、`NTF_ANGLE_DATA`(250Hz)、`NTF_ACC/GYRO/EULER/QUAT`(各 50Hz) 并存，各自采样率正确。
- **有效性说明**：确认磁力计流独立、不挤占其它流。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：通过（2026-11-03 实跑，设备 1D0E）

### MAG-BND-001 setParam("NTF_IMU","OFF") 磁力计停流
- **优先级**：P1
- **测试目的**：验证关闭磁力计后不再有磁力计数据。
- **流程与逻辑**：起流后 `setParam("NTF_IMU","OFF")`，观察磁力计流是否停止。
- **预期结果**：磁力计流停止，无 `NTF_MAG` 数据；不崩溃。
- **有效性说明**：关闭动作正确；因磁力计无独立开关，其它 IMU 流会同时停止（记 SKIP，型号限制）。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：通过（2026-11-03 实跑，设备 1D0E：OFF 后磁力计无新增样本；「只停磁力计」记 SKIP）

### MAG-BND-002 磁力计流参数非法值健壮性
- **优先级**：P1
- **测试目的**：验证非法输入不崩溃。
- **流程与逻辑**：`setParam("NTF_IMU", 非法值)`（如 `"FOO"`）、`setParam("MAG_SAMPLE_RATE", 非法档位)`。
- **预期结果**：返回 `Error` 或忽略，不崩溃、不影响后续正常操作。
- **有效性说明**：参数健壮性。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：通过（2026-11-03 实跑，设备 1D0E）
