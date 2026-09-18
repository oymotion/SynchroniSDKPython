# -*- coding: utf-8 -*-
"""gForceUltra 腕带设备规格（Spec）——单一事实来源。

本文件是该型号「应该支持」的能力基线，测试用例据此硬断言：
设备行为偏离 spec 即 FAIL（而非 SKIP / 自适应跳过）。

同一型号（name_prefix == "gForceUltra"）的多台设备共用本文件，
通过 config.MODEL_SPEC 映射、common.load_spec() 读取，无需逐次确认。

数据来源（实测固化，2026-09-14，设备 80E1，probe_device_info.py dump）：
  - ModelName = 'gForceUltra'，Hardware 1.40，Firmware V1.0.8_2026-09-04_6302890
  - EmgChannelCount = 8；ImpeChannelCount = 8（8 电极接触检测）
  - AccChannelCount = 3；GyroChannelCount = 3；ImuChannelCount = 6（acc3+gyro3，6 轴聚合）
  - Ppg/Spo2/Eeg/Ecg/Brth/MagAngle/Euler/Quat ChannelCount = 0（不支持）
  - EMG_SAMPLE_RATE_LIST = 500|1000，上电默认 1000（EmgMaxSampleRate=1000）
  - IMU_SAMPLE_RATE_LIST = 50（仅 50Hz 一档）
  - 采样率 key 均可 getParam/setParam（见 examples/SynchroniSDKPython_DemoNewMulti.py）

关键判定口径：
  getParam('NTF_XXX') 对几乎全部 key 都返回 'ON'（仅 NTF_PPG_RAW 返回 Error），
  但 'ON' 只表示参数开关被设备接受，不等于该流有数据。
  本文件以 DeviceInfo 的 ChannelCount 为准：ChannelCount > 0 才算支持。
"""

SPEC = {
    # ===== 身份 =====
    "model": "gForceUltra",
    "type": "EMG 腕带（8 通道肌电 + 手势识别 + 6 轴 IMU + 8 通道电极接触检测）",
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
        "NTF_GFORCE_EULER": {"supported": False, "channels": 0},   # 6 轴 IMU 无欧拉角输出
        "NTF_GFORCE_QUAT":  {"supported": False, "channels": 0},   # 6 轴 IMU 无四元数输出
        "NTF_IMU":          {"supported": True,  "channels": 6},   # ImuChannelCount=6（acc3+gyro3 聚合）
        "NTF_MAG_ANGLE":    {"supported": False, "channels": 0},   # 6 轴 IMU 无磁力计
        "NTF_IMPEDANCE":    {"supported": True,  "channels": 8},   # ImpeChannelCount=8（8 电极接触检测）
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
        "FILTER_HPF":  True,   # 10Hz 高通
        "FILTER_LPF":  True,   # 200Hz 低通
    },

    # ===== 采样率参数（EMG/IMU 两套，均可 getParam/setParam）=====
    "sample_rates": {
        "EMG_SAMPLE_RATE": {"rates": ["500", "1000"], "default": "1000"},
        "IMU_SAMPLE_RATE": {"rates": ["50"],          "default": "50"},
    },
}
