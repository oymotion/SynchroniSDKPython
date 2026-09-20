# -*- coding: utf-8 -*-
"""测试设备配置（支持多台设备）。

DEVICES 为设备列表，每台字段：
    name_prefix : 型号标识（如 "OB6000A"/"OB6000C"/"gForceUltra"/"Cerelax"），
                  用作 spec 加载键（common.load_spec），不参与蓝牙扫描匹配
    mac         : 精确 MAC 地址（大写、含冒号）；非空时优先按 MAC 精确匹配
    identity    : 蓝牙地址后四位（如 "6C6B"，对应广播名 "OB6000C(6C6B)" 括号内部分）；
                  非空时按广播名后四位精确匹配

匹配优先级：mac > identity（不再按 name_prefix 前缀匹配，设备广播名经常变化、不可靠）。

当前目标设备：
    TARGET_IDENTITY 指定本轮要测的设备 identity（逗号分隔，可多个），
    各脚本统一从 common.py 读，不要在脚本里硬编码 identity 字符串。
    例：TARGET_IDENTITY = "80F9" 或 "80F9,6C6B"。
"""

# ---- 通用测试参数 ----
SCAN_TIMEOUT_MS = 5000              # 扫描时长（毫秒）
COLLECT_SECONDS = 5                 # 起流后采集时长（秒）
PACKAGE_SAMPLE_COUNT = 20           # init(packageSampleCount, ...)
POWER_REFRESH_INTERVAL_MS = 1000    # init(..., powerRefreshInterval)
MIN_SAMPLES = 1                     # 判定"收到数据"的最小样本数

# ---- 当前目标设备（逗号分隔，脚本统一从 common.py 读）----
TARGET_IDENTITY = "80E1, B383"            # 多设备如 "80F9,6C6B"（当前探测 OB6000A 206F）

# ---- 设备列表（支持多台）----
# 注意：所有目标设备 identity 必须在此列表中，否则 common.py 启动时报错。
DEVICES = [
    {
        "name_prefix": "OB6000C",
        "mac": "78:1C:9D:E4:6C:6B",
        "identity": "6C6B",
    },

    # OB6000A（EEG 采样率支持 250/500 两档，区别于 OB6000C 固定 250）
    {
        "name_prefix": "OB6000A",
        "mac": "",              # TODO: 填入 OB6000A 实际 MAC（可选，用于精确匹配）
        "identity": "XXXX",     # TODO: 填入 OB6000A 广播名后四位 identity（如 "1234"）
    },

    {
        "name_prefix": "OB",
        "mac": "",
        "identity": "2046",
    },

    {
        "name_prefix": "OB6000A",
        "mac": "F0:44:D3:EC:20:6F",
        "identity": "206F",
    },
    {
        "name_prefix": "OB",
        "mac": "78:1C:9D:E4:5B:82",
        "identity": "5B82",
    },
    {
        "name_prefix": "gForceUltra",
        "mac": "BC:93:2A:3F:80:F2",
        "identity": "80F2",
    },
    {
        "name_prefix": "gForceUltra",
        "mac": "BC:93:2A:3F:80:F4",
        "identity": "80F4",
    },
    {
        "name_prefix": "gForceUltra",
        "mac": "BC:93:2A:3F:80:F3",
        "identity": "80F3",
    },
    {
        "name_prefix": "gForceUltra",
        "mac": "BC:93:2A:3F:80:DC",
        "identity": "80DC",
    },

    {
        "name_prefix": "gForceUltra",
        "mac": "F0:44:D3:00:B3:83",
        "identity": "B383",
    },

    {
        "name_prefix": "gForceUltra",
        "mac": "BC:93:2A:3F:80:EF",
        "identity": "80EF",
    },
    {
        "name_prefix": "gForceUltra",
        "mac": "BC:93:2A:3F:80:E5",
        "identity": "80E5",
    },
    {
        "name_prefix": "gForceUltra",
        "mac": "BC:93:2A:3F:80:F9",
        "identity": "80F9",
    },
    {
        "name_prefix": "gForceUltra",
        "mac": "BC:93:2A:3F:80:F8",
        "identity": "80F8",
    },
    {
        "name_prefix": "gForceUltra",
        "mac": "BC:93:2A:3F:80:E1",
        "identity": "80E1",
    },
    {
        "name_prefix": "Cerelax",
        "mac": "BC:93:2A:3F:85:0B",
        "identity": "850B",
    },
    {
        "name_prefix": "Cerelax",
        "mac": "BC:93:2A:3F:85:1C",
        "identity": "851C"
    },
]

# ---- 设备规格映射（name_prefix -> device_specs 下的文件名，不含 .py 后缀）----
# 每个设备型号一份 spec（期望基线），测试脚本用 common.load_spec(name_prefix) 读取。
# 同型号多台设备共用同一份 spec，避免逐次确认。新增型号：复制规格文件 + 在此注册映射。
# 注意：OB 系列已按广播名型号细化（OB6000A/OB6000C 各自 spec）。
#       2046/5B82 型号待探测确认，暂兜底到 ob6000c；OB3000 等其它型号待补。
MODEL_SPEC = {
    "gForceUltra": "gforce_ultra",
    "OB6000A": "ob6000a",
    "OB6000C": "ob6000c",
    "OB": "ob6000c",   # 兜底：未知 OB 型号暂按 OB6000C
    "Cerelax": "cerelax_pro",
}