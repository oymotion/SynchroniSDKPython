# -*- coding: utf-8 -*-
"""测试设备配置（支持多台设备）。

DEVICES 为设备列表，每台字段：
    name_prefix : 广播名前缀（仅作描述/打印用，不参与设备匹配）
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
TARGET_IDENTITY = "6C6B,206F"            # 多设备如 "80F9,6C6B"

# ---- 设备列表（支持多台）----
# 注意：所有目标设备 identity 必须在此列表中，否则 common.py 启动时报错。
DEVICES = [
    {
        "name_prefix": "OB",
        "mac": "78:1C:9D:E4:6C:6B",
        "identity": "6C6B",
    },

    {
        "name_prefix": "OB",
        "mac": "",
        "identity": "2046",
    },

    {
        "name_prefix": "OB",
        "mac": "F0:44:D3:EC:20:6F",
        "identity": "206F",
    },

    {
        "name_prefix": "gForceUltra",
        "mac": "BC:93:2A:3F:80:F3",
        "identity": "80F3",
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
]