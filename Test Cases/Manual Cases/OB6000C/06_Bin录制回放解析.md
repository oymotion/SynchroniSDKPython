# OB6000C Bin 录制/回放/解析用例（BIN-*）

### BIN-FUNC-001 连接后自动生成 .bin
- **测试目的**：验证 bin 自动录制。
- **流程与逻辑**：连接起流，检查是否生成 `{DeviceName}_{MAC}_{时间戳}.bin`。
- **预期结果**：生成 bin 文件。
- **有效性说明**：数据落盘。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：通过

### BIN-FUNC-002 getBinFileInfo 返回字段
- **测试目的**：验证 bin 信息。
- **流程与逻辑**：对有效 bin 调 `getBinFileInfo`。
- **预期结果**：返回含 `device_mac/device_name/chip_type/replay_duration` 的 dict。
- **有效性说明**：bin 元数据完整。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：通过

### BIN-FUNC-003 getBinFileInfo 无效输入返回 None
- **测试目的**：验证逆向。
- **流程与逻辑**：对不存在/无 config 的 bin 调 `getBinFileInfo`。
- **预期结果**：返回 None。
- **有效性说明**：容错。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：通过

### BIN-FUNC-004 回放 realtime=True 按原始节奏
- **测试目的**：验证实时回放。
- **流程与逻辑**：`replayBinFile(path, sensor, realtime=True)`，统计 `onDataCallback` 批数；比对回放是否产生数据、包含 live 的 DataType、startTimeStamp 非 None。
- **预期结果**：回放产生数据，DataType 覆盖 live 类型，startTimeStamp 非 None；回放耗时与录制时长同量级。精确批数可能因 bin 解析器分组方式不同而存在差异，仅作参考。
- **有效性说明**：回放数据保真 + 节奏保真。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：通过

### BIN-FUNC-005 回放 realtime=False 全速
- **测试目的**：验证全速回放。
- **流程与逻辑**：`replayBinFile(path, sensor, realtime=False)`，统计 `onDataCallback` 批数；比对回放是否产生数据、包含 live 的 DataType、startTimeStamp 非 None。
- **预期结果**：回放产生数据，DataType 覆盖 live 类型，startTimeStamp 非 None；回放耗时远小于录制时长（全速）。精确批数可能因 bin 解析器分组方式不同而存在差异，仅作参考。
- **有效性说明**：回放数据保真 + 全速加速。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：通过

### BIN-FUNC-006 pause/resume/stop 回放
- **测试目的**：验证回放控制。
- **流程与逻辑**：回放中依次 `pauseBinReplay`/`resumeBinReplay`/`stopBinReplay`。
- **预期结果**：返回 OK，暂停/恢复/中止生效。
- **有效性说明**：回放可控。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：通过

### BIN-FUNC-007 实时起流中拒绝回放
- **测试目的**：验证互斥。
- **流程与逻辑**：目标 sensor 正在实时起流时调回放。
- **预期结果**：拒绝回放。
- **有效性说明**：状态互斥。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：通过

### BIN-FUNC-008 parseBinToCsv 生成 CSV
- **测试目的**：验证解析导出。
- **流程与逻辑**：`parseBinToCsv`，检查 CSV 行类型与表头。
- **预期结果**：返回 CSV 路径，文件存在；首行为 `timestamp,mac,type,...` 表头；每行 10 列；`type` 列值均在 README 定义内（`raw`/`cmd_send`/`cmd_recv`/`event`/`parsed`），无未知类型。注意：五种行类型出现与否取决于 bin 内容（如 `cmd_recv` 需有命令响应），不全出现不判失败。
- **有效性说明**：数据可分析。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：通过

### BIN-FUNC-009 回放还原 startTimeStamp/delay 与实收一致
- **测试目的**：验证回放保真度。
- **流程与逻辑**：对比回放与实收的 `startTimeStamp/delay`。
- **预期结果**：一致。
- **有效性说明**：回放时间戳还原。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：失败，问题 7101

### BIN-FUNC-010 multiReplayBinFile 多文件同步回放
- **测试目的**：验证 0.9.7 新增的 `multiReplayBinFile` 多 bin 文件按共享时钟对齐同步回放。
- **流程与逻辑**：准备 ≥2 个不同 MAC 的有效 bin 文件；`multiReplayBinFile(file_paths, sensors=None, realtime=True)`；统计各成员 `onDataCallback` 批数与 DataType。
- **预期结果**：返回列表长度等于输入文件数，每个成员返回对应 `SensorProfile`（或 None 表示该成员失败）；各成员均产生回放数据；DataType 覆盖各自 live 类型；全组以最早首条数据记录为 t=0，保留录制时相对偏移。
- **有效性说明**：多设备/多段 bin 的同步回放是 0.9.7 新能力。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：待测试

### BIN-BND-001 默认不导出：临时文件短暂存在后删除
- **测试目的**：验证默认（未设置 `DEBUG_BLE_DATA_PATH`）时 bin 不持久落盘，临时文件在断开后删除。
- **流程与逻辑**：不设置 `DEBUG_BLE_DATA_PATH`，连接起流；起流期间观察系统临时目录是否出现疑似临时 bin；`stopDataNotification`/`disconnect` 后确认日志目录无新增 `.bin`，且此前观察到的临时文件已消失。
- **预期结果**：日志目录不新增 `.bin`；临时文件（若可观测）在断开后消失。
- **有效性说明**：bin 导出默认关闭 + 临时文件生命周期。
- **可自动化**：auto（临时文件观察为 best-effort/informational）
- **人工介入**：无
- **测试结果**：通过

### BIN-BND-002 multiReplayBinFile 无效/重复/不足输入
- **测试目的**：验证多文件回放入参容错。
- **流程与逻辑**：`multiReplayBinFile([])`；`multiReplayBinFile([单文件])`；`multiReplayBinFile(含无效 bin 的列表)`；`multiReplayBinFile(含重复 device_mac 的列表)`。
- **预期结果**：空列表/单文件被拒绝；无效 bin 成员返回 None 不崩溃；重复 MAC 按 SDK 语义处理，其余成员正常回放，不抛异常。
- **有效性说明**：多文件接口容错。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：待测试

### BIN-ROB-001 磁盘不足时跳过/停止录制
- **测试目的**：验证写盘异常处理。
- **流程与逻辑**：模拟磁盘 <100MB 或写盘错误，观察是否跳过/停止且不影响实时流。
- **预期结果**：不崩溃，实时流继续。
- **有效性说明**：资源边界健壮性。
- **可自动化**：auto（通过 icacls 将日志目录设为只读模拟写盘失败，无需实际磁盘不足）
- **人工介入**：无
- **测试结果**：通过

### BIN-ROB-002 multiReplayBinFile 组级 pause/resume/stop 幂等
- **测试目的**：验证多文件回放组级控制与幂等。
- **流程与逻辑**：多文件回放中对任一成员调用 `pauseBinReplay`/`resumeBinReplay`，全组暂停/恢复；`stopBinReplay` 逐成员停止；停止后重复 `stopBinReplay`。
- **预期结果**：pause 任一成员整组暂停；resume 整组恢复；stop 对全部成员生效；重复控制不崩溃、不抛异常。
- **有效性说明**：多文件回放组控制语义（0.9.7 引入）。
- **可自动化**：auto
- **人工介入**：无
- **测试结果**：待测试
