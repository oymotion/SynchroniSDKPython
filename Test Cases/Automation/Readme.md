# 自动化脚本使用说明

## 目录结构：按设备隔离

测试脚本按「设备型号」隔离存放，每个设备一个文件夹，互不干扰：

```
Test Cases/Automation/
├── config.py               # 设备库 + 通用参数 + 当前目标设备（所有脚本共用的唯一配置）
├── smoke_test.py           # 冒烟测试（批量跑 config 中 enabled 的设备）
├── probe_device_info.py    # 设备能力探测（连接后 dump DeviceInfo 全字段）
└── OYWW1100/               # 按设备隔离的脚本目录
    ├── 01_Control_Scan/    #   CTRL-* 控制器与扫描用例
    ├── 02_Connection_State/ #  DEV-SM-* 连接与状态机用例
    └── 03_Dataflow/        #   DATA-* 数据流用例
```

以后要测新设备（如 OB6000C），只需把整套脚本复制到 `OB6000C/` 目录，
并按该设备的实际能力针对性调整（数据源模态、佩戴要求、适用用例等）。
「按设备隔离」换来的是脚本内容可以完全贴合设备特性，互不影响。

## config.py：结构和作用

### 作用

config.py 是**所有自动化脚本共用的唯一配置源**，集中管理三类信息：

1. **通用测试参数**：扫描时长、采集时长、包大小等（所有设备/脚本共用）。
2. **当前目标设备**：本轮要测哪台设备（`TARGET_IDENTITY`）。
3. **设备库**：所有已知设备的匹配信息（`DEVICES` 列表）。

### 字段说明

**通用测试参数：**

| 字段 | 含义 |
|---|---|
| `SCAN_TIMEOUT_MS` | 扫描时长（毫秒） |
| `COLLECT_SECONDS` | 起流后采集时长（秒） |
| `PACKAGE_SAMPLE_COUNT` | `init(packageSampleCount, ...)` 的包样本数 |
| `POWER_REFRESH_INTERVAL_MS` | `init(..., powerRefreshInterval)` 电量刷新周期 |
| `MIN_SAMPLES` | 判定「收到数据」的最小样本数 |

**当前目标设备：**

| 字段 | 含义 |
|---|---|
| `TARGET_IDENTITY` | 本轮要测设备的蓝牙地址后四位（如 `"80F3"`） |

单设备用例统一从 `config.TARGET_IDENTITY` 读取目标，**不要在脚本里硬编码 identity**。
换设备时只需改这一行，例如测 OB6000C 改成 `"6C6B"`。

**设备库（DEVICES 列表）：** 每台设备一条记录：

| 字段 | 含义 |
|---|---|
| `name_prefix` | 广播名前缀（仅作描述/打印用，不参与设备匹配） |
| `mac` | 精确 MAC 地址（大写、含冒号）；非空时优先按 MAC 匹配 |
| `identity` | 蓝牙地址后四位（对应广播名括号内部分，如 `"6C6B"`） |
| `enabled` | 是否参与本轮（多设备用例/批量测试按 enabled 集合匹配） |

**匹配优先级：`mac` > `identity`（不再按 `name_prefix` 前缀匹配，设备广播名经常变化、不可靠）。**

### 两个设备选择机制的定位

| 机制 | 语义 | 适用场景 |
|---|---|---|
| `TARGET_IDENTITY` | 精确指定「测哪一台」 | 单设备用例（大部分脚本） |
| `enabled` | 表达「哪些设备参与本轮」 | 多设备/批量场景（如 DEV-SM-011 多设备同步、smoke_test） |

## 运行方式

单设备用例直接运行脚本即可（目标设备由 `config.TARGET_IDENTITY` 决定）：

```powershell
python OYWW1100/03_Dataflow/test_data_func_006.py
```

多设备用例需先把参与设备在 `config.DEVICES` 中设为 `enabled=True`。
