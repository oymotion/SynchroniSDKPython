# -*- coding: utf-8 -*-
"""OB6000C 脑电传感器设备规格（Spec）——单一事实来源。

本文件是该型号「应该支持」的能力基线，测试用例据此硬断言：
设备行为偏离 spec 即 FAIL（而非 SKIP / 自适应跳过）。

同一型号（name_prefix == "OB6000C"）的多台设备共用本文件，
通过 config.MODEL_SPEC 映射、common.load_spec() 读取，无需逐次确认。

数据来源（实测固化，2026-09-14，设备 6C6B，probe_device_info.py dump）：
  - DeviceName = 'OB6000C(6C6B)'，ModelName = 'OB6000'，Hardware 2.0
  - Firmware V1.0.34_2026-07-29_cfbedac
  - EegChannelCount = 32；ImpeChannelCount = 32（32 电极接触检测）
  - AccChannelCount = 3；GyroChannelCount = 3；EulerChannelCount = 3；QuatChannelCount = 4
  - ImuChannelCount = 13（acc3+gyro3+euler3+quat4 聚合；6 轴 IMU + 融合输出）
  - MagAngleChannelCount = 0（无磁力计）
  - Emg/Ecg/Brth/Ppg/Spo2 ChannelCount = 0（不支持）
  - EEG_SAMPLE_RATE_LIST = 250（固定 250Hz）；IMU_SAMPLE_RATE_LIST = 50（固定 50Hz）
  - 采样率 key 均可 getParam/setParam（见 examples/SynchroniSDKPython_DemoNewMulti.py）

关键判定口径：
  getParam('NTF_XXX') 对几乎全部 key 都返回 'ON'（仅 NTF_PPG_RAW 返回 Error），
  但 'ON' 只表示参数开关被设备接受，不等于该流有数据。
  本文件以 DeviceInfo 的 ChannelCount 为准：ChannelCount > 0 才算支持。

命名说明（勿混用）：
  - 本文件 streams 的 key 采用 setParam/getParam 开关 key（NTF_GFORCE_ACC 等），
    与 gforce_ultra / cerelax_pro 的 spec 一致。
  - 数据流类型标识（getDataType() 返回值）为 DataType.NTF_ACC / NTF_GYRO /
    NTF_EULER_DATA / NTF_QUATERNION / NTF_MAG_ANGLE_DATA，是另一套命名。
"""

SPEC = {
    # ===== 身份 =====
    "model": "OB6000C",
    "type": "脑电传感器（32 通道 EEG + 6 轴 IMU + 32 通道电极接触检测）",
    "name_prefix": "OB6000C",

    # ===== 数据流能力（对应 DeviceInfo 的 ChannelCount 期望）=====
    # supported=True 表示该数据流应存在并产出数据；channels 为期望通道数。
    # supported=False 表示该数据流不应存在（DeviceInfo 对应 ChannelCount==0）。
    "streams": {
        "NTF_EMG":          {"supported": False, "channels": 0},   # EmgChannelCount=0
        "NTF_GEST":         {"supported": False, "channels": 0},   # 脑电设备无手势
        "NTF_GFORCE_ACC":   {"supported": True,  "channels": 3},   # AccChannelCount=3
        "NTF_GFORCE_GYRO":  {"supported": True,  "channels": 3},   # GyroChannelCount=3
        "NTF_GFORCE_EULER": {"supported": True,  "channels": 3},   # EulerChannelCount=3（6 轴融合欧拉角）
        "NTF_GFORCE_QUAT":  {"supported": True,  "channels": 4},   # QuatChannelCount=4（6 轴融合四元数 wxyz）
        "NTF_IMU":          {"supported": True,  "channels": 13},  # ImuChannelCount=13（acc3+gyro3+euler3+quat4）
        "NTF_MAG_ANGLE":    {"supported": False, "channels": 0},   # MagAngleChannelCount=0（无磁力计）
        "NTF_IMPEDANCE":    {"supported": True,  "channels": 32},  # ImpeChannelCount=32（电极接触检测）
        "NTF_EEG":          {"supported": True,  "channels": 32},  # EegChannelCount=32（核心）
        "NTF_ECG":          {"supported": False, "channels": 0},   # EcgChannelCount=0
        "NTF_BRTH":         {"supported": False, "channels": 0},   # BrthChannelCount=0
        "NTF_PPG":          {"supported": False, "channels": 0},   # PpgChannelCount=0
        "NTF_SPO2":         {"supported": False, "channels": 0},   # Spo2ChannelCount=0
    },

    # ===== 滤波开关（FILTER_* 支持 ON/OFF）=====
    "filters": {
        "FILTER_50HZ": True,   # 50Hz 工频陷波
        "FILTER_60HZ": True,   # 60Hz 工频陷波
        "FILTER_HPF":  True,   # 高通
        "FILTER_LPF":  True,   # 低通
    },

    # ===== 采样率参数（EEG/IMU 两套，均可 getParam/setParam）=====
    "sample_rates": {
        "EEG_SAMPLE_RATE": {"rates": ["250"], "default": "250"},
        "IMU_SAMPLE_RATE": {"rates": ["50"],  "default": "50"},
    },
}
