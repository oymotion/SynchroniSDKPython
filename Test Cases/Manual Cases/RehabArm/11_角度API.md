# RehabArm 角度数据流用例（ANGLE-*）

> 本章是 RehabArm 的**专项补充章（编号 11）**，聚焦「角度」这一特殊能力。
> 按设备实际能力执行：`getDeviceInfo()` 对应 `AngleChannelCount>0` 才执行；否则记录“不支持，跳过”。

## 设备角度能力基线（来自 probe_device.py 实测，2026-10-27，设备 1D0E）

| 项 | 值 |
|----|----|
| 能力声明 | `DeviceInfo.AngleChannelCount=1`、`AngleSampleRate=250` |
| 数据流类型 | `DataType.NTF_ANGLE_DATA`（0x0D） |
| 通道数 | 1（单通道角度值） |
| 采样率 | 250 Hz（固定，不可设置） |
| 单位 | deg（度） |
| 量程 | 约 [锐角值, 180°]（展开≈180° 为上限，屈起≈锐角值<90° 为下限，从 180° 向下递减；具体下限值待与固件确认） |
| 显示标签 | `Angle (deg)`（即“角度，单位度”） |
| 流开关参数 | `getParam("NTF_ANGLE_DATA")` / `setParam("NTF_ANGLE_DATA", "ON"/"OFF")` |
| 采样率参数 | 固定 250Hz，`ANGLE_SAMPLE_RATE` 不可设置 |

> 注意：角度流独立于 IMU 聚合流（`ImuChannelCount=13` = acc3+gyro3+euler3+quat4），
> 角度(1ch/250Hz) 与磁力计(3ch/10Hz) 均为独立流，不并入 IMU。

---

### ANGLE-FUNC-001 DeviceInfo 声明角度能力
- **测试目的**：验证设备通过 `DeviceInfo` 正确声明角度能力。
- **流程与逻辑**：Ready 后 `init`，`fetchDeviceInfo()` 或 `getDeviceInfo()` 读 `AngleChannelCount`、`AngleSampleRate`。
- **预期结果**：`AngleChannelCount==1` 且 `AngleSampleRate==250`。
- **有效性说明**：能力声明是后续所有角度用例的前置判定依据（`>0` 才执行）。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：待执行

### ANGLE-FUNC-002 角度流开关 getParam/setParam
- **测试目的**：验证角度流开关参数可读写、可切换。
- **流程与逻辑**：`getParam("NTF_ANGLE_DATA")` 读当前状态；`setParam("NTF_ANGLE_DATA","ON")` / `setParam("NTF_ANGLE_DATA","OFF")` 各切换一次并回读。
- **预期结果**：`getParam` 返回非 `Error` 开头；`ON`/`OFF` 切换后状态一致（返回 `OK` 或对应状态值）。
- **有效性说明**：开关是控制角度流起停的正规入口。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：待执行

### ANGLE-FUNC-003 起流后收到 NTF_ANGLE_DATA 且 getDataType 匹配
- **测试目的**：验证角度流真实产出数据，类型正确。
- **流程与逻辑**：开 `NTF_ANGLE_DATA` → `startDataNotification()` → 在 `onDataCallback` 统计 `getDataType()`。
- **预期结果**：收到 `DataType.NTF_ANGLE_DATA`（0x0D）数据批。
- **有效性说明**：证明角度流不是“仅参数接受、无实际数据”。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：待执行

### ANGLE-FUNC-004 角度 SensorData 元数据合法
- **测试目的**：验证角度批次的元数据接口。
- **流程与逻辑**：对 `NTF_ANGLE_DATA` 批次检查 `getChannelCount()`、`getSampleRate()`、`getSampleCount()`、`isDataValid()`。
- **预期结果**：`getChannelCount()==1`、`getSampleRate()==250`、`getSampleCount()>0`、`isDataValid()==True`。
- **有效性说明**：元数据是上层解析角度值的依据。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：待执行

### ANGLE-FUNC-005 角度单点访问器一致、sampleIndex 单调
- **测试目的**：验证角度样本的单点访问接口一致性与索引单调性。
- **流程与逻辑**：对 `NTF_ANGLE_DATA` 用 `getData(0, si)` / `getRawData(0, si)` / `getSampleIndex(0, si)` / `getChannelSample(0, si)` 逐点访问。
- **预期结果**：各访问器一致（`getData` 与 `getChannelSample(0,si).data` 相等），`sampleIndex` 单调递增，无越界。
- **有效性说明**：细粒度访问器正确。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：待执行

### ANGLE-FUNC-006 角度值物理合理性（展开 180°、屈起为锐角）
- **测试目的**：验证角度值语义（单位、量程、方向）。
- **流程与逻辑**：设备保持某已知姿态，读 `getData(0, si)` 的角度值；转动到另一姿态再读，观察数值变化方向与幅度。
- **预期结果**：单位为 deg；展开（伸直）≈180° 为上限、屈起≈锐角值（<90°）为下限，即范围约 [锐角值, 180°]、从 180° 向下递减；数值方向与屈伸一致；具体下限值/参考方向待与固件确认。
- **有效性说明**：确认角度值可解释、可被上层算法直接使用。
- **可自动化**：semi-auto
- **人工介入**：人工摆放设备到已知姿态/角度，记录基准值与变化。
- **测试结果**：待执行

### ANGLE-FUNC-007 角度采样率固定 250Hz（不可设置）
- **测试目的**：验证角度采样率固定为 250Hz、不可设置。
- **流程与逻辑**：`getParam("ANGLE_SAMPLE_RATE")`、`getParam("ANGLE_SAMPLE_RATE_LIST")`；并 `setParam("ANGLE_SAMPLE_RATE", 非 250 档位)` 看是否拒绝。
- **预期结果**：采样率固定 250Hz；`getParam` 返回 `Error` 或仅 250，`setParam` 改档位不生效/返回 Error。
- **有效性说明**：确认角度采样率不可改（demo 中角度无采样率选择，与 IMU 50Hz 相互独立）。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：待执行

### ANGLE-FUNC-008 角度流与 IMU 四路流共存、采样率独立
- **测试目的**：验证角度流与 acc/gyro/euler/quat 四路流可同时起流、采样率互不干扰。
- **流程与逻辑**：同时开启角度流与 IMU 流，起流后统计各流 `getDataType()` 与 `getSampleRate()`。
- **预期结果**：`NTF_ANGLE_DATA`(250Hz) 与 `NTF_ACC/NTF_GYRO/NTF_EULER_DATA/NTF_QUATERNION`(各 50Hz) 并存，各自采样率正确。
- **有效性说明**：确认角度流不挤占、不依赖 IMU 流。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：待执行

### ANGLE-BND-001 setParam("NTF_ANGLE_DATA","OFF") 停流干净
- **测试目的**：验证关闭角度流后不再有角度数据。
- **流程与逻辑**：起流后 `setParam("NTF_ANGLE_DATA","OFF")`，观察角度流是否停止、其它流是否不受影响。
- **预期结果**：角度流停止，无 `NTF_ANGLE_DATA` 数据；不崩溃。
- **有效性说明**：关闭动作正确且不影响其它流。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：待执行

### ANGLE-BND-002 角度流参数非法值健壮性
- **测试目的**：验证非法输入不崩溃。
- **流程与逻辑**：`setParam("NTF_ANGLE_DATA", 非法值)`（如 `"FOO"`）、`setParam("ANGLE_SAMPLE_RATE", 非法档位)`。
- **预期结果**：返回 `Error` 或忽略，不崩溃、不影响后续正常操作。
- **有效性说明**：参数健壮性。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：待执行
