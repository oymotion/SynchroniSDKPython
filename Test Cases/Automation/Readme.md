# Python SDK 自动化测试（Automation）说明

> 本文档面向接手本仓库 Python 自动化测试的人，交代「设计 / 架构 / 怎么跑 / 怎么加用例 / 怎么接新设备」。
> 上层通用约定（统一接口范式、AGENTS.md 的项目级说明）见仓库根目录 `AGENTS.md`，本文档专注 `Automation/` 这一层。

---

## 1. 定位与目标

`Automation/` 是一套 **spec 驱动、能力不硬编码** 的自动化测试框架，用于：

- 对每一台传感器设备，跑通「扫描 → 连接 → init → 起流 → 参数 → 回放/日志/电量 → 多设备 → 异步」全链路能力验证；
- 用 `DeviceInfo`（设备实际能力）+ `device_specs/*.py`（期望基线）做运行时判定，**不对具体数值（通道数 / 采样率档位 / 型号）硬编码进测试代码**；
- 同型号多台设备共用同一份 spec，只改 `config.py` 的目标设备即可切换被测对象。

核心诉求（同 AGENTS.md）：**SDK 的 API 要对「设备能力」做全覆盖的适配与测试**——每种能力（通道数、采样率、支持的流、支持的参数）都被 SDK 正确表达，并被用例覆盖。

---

## 2. 目录结构

```
Automation/
├── config.py                # ①目标设备 + ②通用参数 + ③spec 映射（单一配置入口）
├── common.py                # 共享工具：spec 加载 / 扫描匹配 / record 三态 / bin 兼容 shim
├── probe_device.py          # 通用三层能力探测（新设备接入第一步）
├── probe_device_info.py     # DeviceInfo 单层探测
├── probe_data_types.py      # DataType 起流观测探测
├── probe_saturation.py      # 饱和度探测
├── replay_worker.py         # bin 回放子进程 worker（summary / control 两模式）
├── smoke_test.py            # 冒烟脚本
├── _debug_battery.py        # 电量调试
│
├── device_specs/            # 每型号一份规格（单一事实来源，期望基线）
│   ├── __init__.py
│   ├── cerelax_pro.py  gforce_ultra.py  gforce_max.py  gforce_oct_plus.py
│   ├── ob3000.py  ob5000.py  ob6000a.py  ob6000c.py  rehab_arm.py
│
├── Generalized/             # spec 驱动的通用自动化脚本（跨型号复用）
│   ├── 01_Control_Scan/               # 控制器与扫描
│   ├── 02_Connection_State/           # 连接与状态机
│   ├── 03_Dataflow/                   # 数据流
│   ├── 04_Parameters/                 # 参数
│   ├── 05_MultiDevice/                # 多设备同步
│   ├── 06_Bin_Record_Replay/          # bin 录制回放解析
│   ├── 07_Battery_Log/                # 电量日志调试
│   ├── 09_Async_Interface/            # 异步接口
│   ├── 10_Base_Interface_and_Boundary/# 底层接口与边界补充
│   ├── 11_Angle/ 12_Magnetometer/     # 角度 / 磁力计（新增专项章节）
│   ├── e2e_smoke.py                   # 冒烟
│   ├── longevity.py                   # 长稳
│   ├── stress_connect_stream.py       # 连接/起流压力
│   └── test_measure_sample_rate.py    # 采样率实测
│
├── Cerelax Pro/             # 设备专项脚本（e2e/longevity/stress/采样率/record_bin/triage）
└── Bug Verification/        # bug 复现验证脚本（如 test_4662 阻抗假 0）
```

> 说明：`08_人工硬件用例` 无对应目录（人工硬件用例无法自动化，见 AGENTS.md §4.2）。

---

## 3. 核心设计：spec 驱动 + 能力不硬编码

### 3.1 单一事实来源

- **设备实际能力**：`DeviceInfo`（运行时由 SDK 返回），字段形如 `XxxChannelCount` / `XxxSampleRate`。
- **期望基线**：`device_specs/<model>.py` 里的 `SPEC` dict，是「这个型号应该支持什么」的权威声明。
- 测试逻辑 **只读 spec + DeviceInfo 运行时结果** 做断言：`supported=True 且 channels>0` 才测对应能力，否则记 `SKIP`（「不支持，跳过」），不硬编码具体数值。

### 3.2 SPEC 结构（device_specs 里的 `SPEC` dict）

```python
SPEC = {
    "model": "gForceOct+",          # 型号名
    "type": "EMG 腕带（…）",         # 一句话描述
    "name_prefix": "gForceOct+",    # spec 加载键（config.MODEL_SPEC 的 key）
    "streams": {                    # 数据流能力：supported=True 且 channels>0 才算支持
        "NTF_EMG":        {"supported": True,  "channels": 8},
        "NTF_EEG":        {"supported": False, "channels": 0},  # 不支持
        "NTF_IMU":        {"supported": True,  "channels": 13},
    },
    "imu_sub_channels": {           # 聚合流子通道组成（无则省略）
        "AccChannelCount": 3, "GyroChannelCount": 3,
        "EulerChannelCount": 3, "QuatChannelCount": 4,
    },
    "filters": {                    # 滤波开关（空 dict = 无）
        "FILTER_50HZ": True, "FILTER_60HZ": True,
    },
    "sample_rates": {               # 采样率参数
        "EMG_SAMPLE_RATE": {
            "rates": ["500", "1000"], "default": "1000",
            "stream": "NTF_EMG", "settable": True, "co_stream": None,
        },
        "IMPEDANCE": {
            "rates": ["1"], "default": "1",
            "stream": "NTF_IMPEDANCE", "settable": False, "co_stream": "NTF_EMG",
        },
    },
}
```

各字段含义：

| 字段 | 含义 |
|------|------|
| `streams[NTF_X].supported` | 该流是否应存在并产出数据（True 且 channels>0） |
| `streams[NTF_X].channels` | 期望通道数 |
| `imu_sub_channels` | 聚合流（如 NTF_IMU）的子通道组成 |
| `filters` | 滤波开关（`FILTER_*` 支持 ON/OFF） |
| `sample_rates[X]` | 采样率参数：`rates` 档位、`default` 默认档、`stream` 所属流、`settable` 是否可设、`co_stream` 伴随流（需同起才有数据）、`sub_streams` 聚合流拆分成的子 DataType |

> 注意 `sample_rates` 里的 `stream` 是 **流开关 key + DataType 类型名**；而 `setParam`/`getParam` 的流开关 key 与 DataType 枚举名并非一一对应（详见 §11 踩坑）。

---

## 4. config.py（唯一配置入口）

三个职责：

```python
# ① 通用参数
SCAN_TIMEOUT_MS = 5000
COLLECT_SECONDS = 5
PACKAGE_SAMPLE_COUNT = 20
POWER_REFRESH_INTERVAL_MS = 1000
MIN_SAMPLES = 1

# ② 当前目标设备（identity 逗号分隔；脚本统一从 common.py 读，勿在脚本里硬编码）
TARGET_IDENTITY = "80AF"    # gForceOct+

# ③ 设备列表（name_prefix / mac / identity）
DEVICES = [
    {"name_prefix": "gForceOct+", "mac": "BC:93:2A:3F:80:AF", "identity": "80AF"},
    ...
]

# ④ 型号 → spec 文件映射（不含 .py）
MODEL_SPEC = {
    "gForceOct+": "gforce_oct_plus",
    "Cerelax-Pro": "cerelax_pro",
    ...
}
```

关键约定：

- **匹配优先级**：`mac`（精确）> `identity`（广播名括号内后四位，如 `gForceOct+(80AF)` → `80AF`）。不再按 `name_prefix` 前缀匹配（广播名经常变化不可靠）。
- **`TARGET_IDENTITY` 里的 identity 必须已在 `DEVICES` 中登记**，否则 `common.py` 导入时直接 `SystemExit` 报错。
- 命令行可覆盖目标：`python <script>.py 80AF,9A91`（`common.resolve_target_identity()`）。
- 新增型号 = ①`DEVICES` 加一条 + ②`MODEL_SPEC` 加映射 + ③新建 `device_specs/<snake_case>.py`。

---

## 5. common.py（共享工具速查）

测试脚本统一 `import common`，核心 API：

| 函数 | 作用 |
|------|------|
| `common.TARGET_IDENTITIES` | 已解析并校验的目标 identity 列表（模块加载时算好） |
| `common.load_spec(name_prefix)` | 按型号名读 spec，返回 `SPEC` dict |
| `common.spec_for_identity(identity)` | 由 identity 反查 name_prefix 再加载 spec |
| `common.identity_of_device(device)` | 从广播名后四位（优先）/ MAC（兜底）取 identity |
| `common.match_target(devices, target_identity=None)` | 从扫描结果匹配第一台目标（mac > identity） |
| `common.match_all_targets(devices)` | 匹配所有目标，返回 `[(identity, device)]` |
| `common.scan_and_match(ctrl, ...)` | 扫描+匹配，未命中自动重试（默认 3 次/间隔 10s） |
| `common.async_scan_and_match(...)` | 异步版（asyncio 内使用） |
| `common.scan_and_match_all(...)` / `async_scan_and_match_all(...)` | 匹配全部目标 |
| `common.record(results, name, ok, expect, actual)` | 记录一条结果（PASS/FAIL/SKIP） |
| `common.setup_sanity_log(base_dir, ...)` | 把 stdout tee 到日志文件（长稳/压力用） |
| `common.bin_info_to_dict(info)` | 兼容 SDK 1.3.9 `getBinFileInfo` dict→对象破坏性变更 |

还有两个「隐式」行为在 import `common` 时自动生效：

1. `_parse_target_identities()`：校验 `TARGET_IDENTITY` 合法性。
2. `_install_bin_info_compat_shim()`：给 `SensorController.getBinFileInfo` 包一层兼容 shim，让旧脚本的 `.get("device_mac")` 继续可用（幂等，只包一次）。

---

## 6. Generalized 通用脚本约定

### 6.1 目录/命名映射

脚本命名 `test_{模块}_{类型}_{编号}.py`（类型 ∈ `func`/`bnd`/`rob`/`sm`/`perf`；编号 = 手工 ID 后三位）：

| 目录 | 手工章节 | 模块 | 手工 ID 前缀 |
|---|---|---|---|
| 01_Control_Scan | 01 控制器与扫描 | ctrl | CTRL-FUNC / CTRL-BND / CTRL-ROB |
| 02_Connection_State | 02 连接与状态机 | dev | DEV-SM |
| 03_Dataflow | 03 数据流 | data | DATA-FUNC / DATA-BND / DATA-PERF |
| 04_Parameters | 04 参数 | param | PARAM-FUNC / PARAM-BND / PARAM-ROB |
| 05_MultiDevice | 05 多设备同步 | multi | MULTI-FUNC |
| 06_Bin_Record_Replay | 06 bin 录制回放解析 | bin | BIN-FUNC / BIN-BND / BIN-ROB |
| 07_Battery_Log | 07 电量日志调试 | batt / log | BATT-FUNC / LOG-FUNC |
| 09_Async_Interface | 09 异步接口 | async | ASYNC-FUNC |
| 10_Base_Interface_and_Boundary | 10 底层接口与边界 | misc | MISC-FUNC / MISC-BND |
| 11_Angle / 12_Magnetometer | 角度 / 磁力计（新增） | angle / mag | — |

### 6.2 用例脚本骨架（以 `test_ctrl_func_001.py` 为例）

```python
import os, sys
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOMATION_DIR = os.path.dirname(os.path.dirname(BASE_DIR))
sys.path.insert(0, AUTOMATION_DIR)          # 保证能 import config/common

from sensor import *
import config, common
from common import record

def main():
    ctrl = SensorControllerInstance
    results = []
    # ... 扫描/连接/断言，每条断言用 record(...)
    # 汇总：PASS / FAIL / SKIP
    ctrl.terminate()

if __name__ == "__main__":
    main()
```

### 6.3 五条铁律（新增/改用例务必遵守）

1. **能力来源只认 spec**：删硬编码的 DeviceInfo 字段 / 流键 / 采样率档位 / 通道数 / 型号判定，改读 `spec["streams"]/["filters"]/["sample_rates"]`。
2. **载入 spec**：脚本内加 helper（注意 `identity is None` 与异常兜底）：

   ```python
   def _load_spec(target):
       identity = common.identity_of_device(target)
       if identity is None:
           return None
       try:
           return common.spec_for_identity(identity)
       except Exception:
           return None
   ```

3. **目标匹配**：统一走 `common.match_target` / `scan_and_match` / `match_all_targets`，不硬编码 MAC / 型号前缀。
4. **record 三态**：`ok=True` PASS / `False` FAIL / `None` SKIP。
5. **错误判定三态**（SDK 1.4.3 语义）：字符串以 `Error` 开头 / `抛异常` 开头 / `<` 开头（如 `<IOError: …>`）都视为「错误 / 被拒绝」；成功只认 `== "OK"`。
6. **措辞不硬编码型号名**：docstring / print / input 里不写 gForceUltra 等型号，改「待测设备」。

---

## 7. 专项脚本

| 脚本 | 作用 | 说明 |
|------|------|------|
| `probe_device.py <identity>` | 通用三层能力探测 | ①DeviceInfo 全字段 ②getParam 逐 key ③起流观测 DataType |
| `probe_device_info.py` / `probe_data_types.py` / `probe_saturation.py` | 单层探测 | DeviceInfo / DataType / 饱和度 |
| `replay_worker.py` | bin 回放子进程 | `--mode summary`（统计） / `--mode control`（暂停恢复） |
| `smoke_test.py` | 冒烟 | |
| `Generalized/e2e_smoke.py` | 端到端冒烟 | |
| `Generalized/longevity.py` | 长稳 | 长时间起流观测 |
| `Generalized/stress_connect_stream.py` | 连接/起流压力 | |
| `Generalized/test_measure_sample_rate.py` | 采样率实测 | 遍历 spec 的 `sample_rates` |

> `replay_worker.py` 必须用 **fresh controller + fresh profile** 子进程回放，不能复用已连接/已起流的 profile，否则回放不泵数据（见 AGENTS.md §9.2）。

---

## 8. 如何执行

在 `Automation/` 目录下（Python 已装 `sensor` 包，见 `pip show sensor` 定位；Python 3.13/3.14 均可）：

```powershell
cd "Python/SynchroniSDKPython/Test Cases/Automation"

# 探测某台设备能力
python probe_device.py 80AF

# 跑单个通用用例（可带 identity 覆盖 config 默认）
python Generalized/01_Control_Scan/test_ctrl_func_001.py

# 采样率实测
python Generalized/test_measure_sample_rate.py

# 冒烟 / 长稳 / 压力
python Generalized/e2e_smoke.py
python Generalized/longevity.py
python Generalized/stress_connect_stream.py

# 校验 spec 可编译 / 可载入
python -m py_compile "device_specs/gforce_oct_plus.py"
python -c "import common; print(common.load_spec('gForceOct+')['model'])"
```

> 通用脚本可直接运行（各自把 `AUTOMATION_DIR` 加进 `sys.path`）；无需额外启动器。

---

## 9. 新设备接入流程（一句话适配）

见技能 `sensor-sdk-onboard-device`，核心 6 步：

1. **解析输入**：MAC（大写含冒号）+ 型号名；identity 先用 MAC 后四位，扫描后从广播名括号内提取真实值覆盖。
2. **登记 config.py**：`DEVICES` 加一条 + `MODEL_SPEC` 加映射 + `TARGET_IDENTITY` 切到新 identity。
3. **跑探测**：`python probe_device.py <identity>`。
4. **解读**：三层交叉判定每个能力（流 supported / channels / 采样率档位 / 滤波），不硬编码。
5. **生成 spec**：新建 `device_specs/<snake_case>.py`，docstring 里记录三层实测数据来源。
6. **校验**：`py_compile` + `common.load_spec()`。

> 采样率档位陷阱：`*_SAMPLE_RATE_LIST` 可能只返回当前档；要交叉 `XMaxSampleRate` 判断是否多档，必要时 `setParam` 实测确认（详见 onboard skill）。

---

## 10. Bug 复现验证

`Bug Verification/` 放一次性 bug 复现/验证脚本，如 `test_4662.py`（阻抗「假 0」）。这类脚本不进入 `Generalized/` 正式用例，验证完保留作为回归参考。

---

## 11. 关键踩坑（写用例前必看）

> 完整版见 AGENTS.md §9，这里只列最容易踩的：

- **`setParam`/`getParam` 的流开关 key ≠ DataType 枚举名**：权威 key 取自 `getParam("NTF")` 返回。角度流开关是 `NTF_ANGLE` 而非 `NTF_ANGLE_DATA`；磁力计无独立开关、随 `NTF_IMU` 聚合起停。写脚本时把 `STREAM_KEY`（getDataType 匹配/spec 流键）与 `SET_KEY`（setParam key）分开定义。
- **通道编号 0 起**：`getChannelCount()` 返回 N 时有效索引 `0..N-1`。
- **多流通道隔离**：各流 `channelIndex` 独立编号，统计前先按 `getDataType()` 分桶；EMG ch0 / EEG ch0 / IMU ch0 是不同物理通道。
- **阻抗只在 `NTF_IMPEDANCE`/`NTF_IMPEDANCE_EXT` 流有值**，其它流的 `Sample.impedance` 恒为 0.0。
- **采样率/样本统计务必按 `getDataType()` 分桶**，别把所有流加一起（EEG+IMU 会虚高）。
- **`replayBinFile` 不能复用已连接/已起流的 profile**，必须 fresh controller + fresh profile（`replay_worker.py`）。
- **回放 `startTimeStamp` bug 7101**：回放会重新生成 startTimeStamp，与 live 不一致（真 SDK bug，非脚本问题）。
- **`getBinFileInfo` 兼容**：SDK 1.3.9 由 dict 改成 `BinFileInfo` 对象，`common` 已自动装 shim 兼容。
- **GEST 与 EMG 高采样冲突**：测 EMG 高采样率时先 `setParam("NTF_GEST","OFF")`，别当硬规则硬编码进断言。

---

## 12. 交接速览（TL;DR）

- **改被测设备** → 只改 `config.py` 的 `TARGET_IDENTITY`（identity 必须已在 `DEVICES` 登记）。
- **加一个新用例** → 在 `Generalized/<对应章节>/` 复制同类脚本改编号，读 spec、用 `record`、不硬编码能力/型号。
- **加一个新型号** → 走 onboard 流程（登记 → 探测 → 写 spec → 校验）。
- **spec 是唯一事实来源**，型号差异只体现在 spec/config，不改测试代码。
- **跑不动/行为诡异先查 AGENTS.md §9**，那是最新踩坑沉淀。
