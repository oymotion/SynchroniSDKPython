# -*- coding: utf-8 -*-
"""gForceUltra 腕带设备规格（Spec）——单一事实来源。

本文件是该型号「应该支持」的能力基线，测试用例据此硬断言：
设备行为偏离 spec 即 FAIL（而非 SKIP / 自适应跳过）。

同一型号（name_prefix == "gForceUltra"）的多台设备共用本文件，
通过 config.MODEL_SPEC 映射、common.load_spec() 读取，无需逐次确认。

数据来源（一次固化，后续只改本文件）：
  - EMG 8 通道 / 1000Hz（最大）：官方产品页 https://oymotion.com/product17
  - 6 轴 IMU（3 轴加速度 + 3 轴陀螺仪）：官方产品页
  - ACC/GYRO/EULER/QUAT 通道布局：README DataType.NTF_IMU 聚合批
    (acc 0-2 / gyro 3-5 / euler 6-8 / quat 9-12)
  - NTF_IMU 为四路 NTF_GFORCE_* 的 master switch：README setParam 文档
"""

SPEC = {
    # ===== 身份 =====
    "model": "gForceUltra",
    "type": "EMG 腕带（8 通道肌电 + 手势识别 + 6 轴 IMU）",
    "name_prefix": "gForceUltra",

    # ===== 数据流能力（对应 DeviceInfo 的 ChannelCount 期望）=====
    # supported=True 表示该数据流应存在并产出数据；channels 为期望通道数。
    # supported=False 表示该数据流不应存在（DeviceInfo 对应 ChannelCount==0，
    #   起流后也不应收到该 DataType 数据）。
    "streams": {
        "NTF_EMG":          {"supported": True,  "channels": 8},   # 8 通道干电极肌电
        "NTF_GEST":         {"supported": True,  "channels": 1},   # 手势 ID（单值流）
        "NTF_GFORCE_ACC":   {"supported": True,  "channels": 3},   # 加速度 3 轴
        "NTF_GFORCE_GYRO":  {"supported": True,  "channels": 3},   # 陀螺仪 3 轴
        "NTF_GFORCE_EULER": {"supported": True,  "channels": 3},   # 欧拉角 3 轴
        "NTF_GFORCE_QUAT":  {"supported": True,  "channels": 4},   # 四元数 w/x/y/z
        # NTF_IMU 无独立聚合数据流（传统 EMG 设备 ImuChannelCount==0）；
        # 它只是四路 NTF_GFORCE_* 的 master switch（README setParam 文档）。
        "NTF_IMU":          {"supported": False, "channels": 0},
        "NTF_MAG_ANGLE":    {"supported": False, "channels": 0},   # 6 轴 IMU 无磁力计
        "NTF_IMPEDANCE":    {"supported": False, "channels": 0},   # 腕带干电极无阻抗接触检测
        "NTF_EEG":          {"supported": False, "channels": 0},   # 腕带无 EEG
        "NTF_ECG":          {"supported": False, "channels": 0},   # 腕带无 ECG
        "NTF_BRTH":         {"supported": False, "channels": 0},   # 腕带无呼吸
        "NTF_PPG":          {"supported": False, "channels": 0},   # 腕带无 PPG（NTF_PPG_RAW 为其别名）
        "NTF_SPO2":         {"supported": False, "channels": 0},   # 腕带无 SpO2
    },

    # ===== 滤波开关（FILTER_* 支持 ON/OFF）=====
    "filters": {
        "FILTER_50HZ": True,   # 50Hz 工频陷波
        "FILTER_60HZ": True,   # 60Hz 工频陷波
        "FILTER_HPF":  True,   # 0.5Hz 高通
        "FILTER_LPF":  True,   # 80Hz 低通
    },

    # ===== 核心采样率参数 =====
    "sample_rate": {
        "key": "EMG_SAMPLE_RATE",   # 采样率参数 key
        "rates": ["500", "1000"],   # 合法档位
        "default": "1000",          # 上电默认值（未 set 前首次读取）
    },
}
