# -*- coding: utf-8 -*-
"""Cerelax 脑电头环设备规格（Spec）——单一事实来源。

本文件是该型号「应该支持」的能力基线，测试用例据此硬断言：
设备行为偏离 spec 即 FAIL（而非 SKIP / 自适应跳过）。

同一型号（name_prefix == "Cerelax"）的多台设备共用本文件，
通过 config.MODEL_SPEC 映射、common.load_spec() 读取，无需逐次确认。

数据来源（实测固化，2026-09-14，设备 851C，probe_device_info.py dump）：
  - ModelName = 'Cerelax-Pro'，Firmware V1.0.3
  - EegChannelCount = 2；PpgChannelCount = 2；Spo2ChannelCount = 2
  - ImpeChannelCount = 2；AccChannelCount = 3；GyroChannelCount = 3
  - ImuChannelCount = 6（= acc 3 + gyro 3，6 轴聚合，无磁力计）
  - Emg/Ecg/Brth/MagAngle/Euler/Quat ChannelCount = 0（不支持）
  - EEG_SAMPLE_RATE_LIST = 250|500|1000，当前/默认 1000
  - IMU_SAMPLE_RATE_LIST = 50|100|200，当前/默认 200
  - PPG_SAMPLE_RATE_LIST = 50|100|200|400，当前/默认 50
  - 采样率 key 均可 getParam/setParam（见 examples/SynchroniSDKPython_DemoNewMulti.py）

关键判定口径：
  getParam('NTF_XXX') 对几乎全部 key 都返回 'ON'（仅 NTF_PPG_RAW 返回 Error），
  但 'ON' 只表示参数开关被设备接受，不等于该流有数据。
  本文件以 DeviceInfo 的 ChannelCount 为准：ChannelCount > 0 才算支持。
"""

SPEC = {
    # ===== 身份 =====
    "model": "Cerelax-Pro",
    "type": "脑电头环（2 通道 EEG + 2 通道 PPG + 6 轴 IMU 聚合 + 电极接触检测）",
    "name_prefix": "Cerelax",

    # ===== 数据流能力（对应 DeviceInfo 的 ChannelCount 期望）=====
    # supported=True 表示该数据流应存在并产出数据；channels 为期望通道数。
    # supported=False 表示该数据流不应存在（DeviceInfo 对应 ChannelCount==0）。
    "streams": {
        "NTF_EMG":          {"supported": False, "channels": 0},   # EmgChannelCount=0
        "NTF_GEST":         {"supported": False, "channels": 0},   # 头环无手势
        "NTF_GFORCE_ACC":   {"supported": True,  "channels": 3},   # AccChannelCount=3
        "NTF_GFORCE_GYRO":  {"supported": True,  "channels": 3},   # GyroChannelCount=3
        "NTF_GFORCE_EULER": {"supported": False, "channels": 0},   # EulerChannelCount=0（6 轴无欧拉角）
        "NTF_GFORCE_QUAT":  {"supported": False, "channels": 0},   # QuatChannelCount=0（6 轴无四元数）
        "NTF_IMU":          {"supported": True,  "channels": 6},   # ImuChannelCount=6（acc3+gyro3 聚合）
        "NTF_MAG_ANGLE":    {"supported": False, "channels": 0},   # MagAngleChannelCount=0（无磁力计）
        "NTF_IMPEDANCE":    {"supported": True,  "channels": 2},   # ImpeChannelCount=2（电极接触检测）
        "NTF_EEG":          {"supported": True,  "channels": 2},   # EegChannelCount=2（核心）
        "NTF_ECG":          {"supported": False, "channels": 0},   # EcgChannelCount=0
        "NTF_BRTH":         {"supported": False, "channels": 0},   # BrthChannelCount=0
        "NTF_PPG":          {"supported": True,  "channels": 2},   # PpgChannelCount=2
        "NTF_SPO2":         {"supported": True,  "channels": 2},   # Spo2ChannelCount=2
    },

    # ===== 滤波开关（FILTER_* 支持 ON/OFF）=====
    "filters": {
        "FILTER_50HZ": True,   # 50Hz 工频陷波
        "FILTER_60HZ": True,   # 60Hz 工频陷波
        "FILTER_HPF":  True,   # 高通
        "FILTER_LPF":  True,   # 低通
    },

    # ===== 采样率参数（EEG/IMU/PPG 三套，均可 getParam/setParam）=====
    "sample_rates": {
        "EEG_SAMPLE_RATE": {"rates": ["250", "500", "1000"],       "default": "1000"},
        "IMU_SAMPLE_RATE": {"rates": ["50", "100", "200"],         "default": "200"},
        "PPG_SAMPLE_RATE": {"rates": ["50", "100", "200", "400"],  "default": "50"},
    },
}
