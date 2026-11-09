# -*- coding: utf-8 -*-
"""OB5000 生物电传感器设备规格（Spec）——单一事实来源。

本文件是该型号「应该支持」的能力基线，测试用例据此硬断言：
设备行为偏离 spec 即 FAIL（而非 SKIP / 自适应跳过）。

数据来源（实测固化，2026-11-04，设备 6DEF，probe_device.py 三层探测）：
  - DeviceName = 'OB5000(6DEF)'，ModelName = 'OB5000'，Hardware 3.27
  - Firmware V1.0.41_2026-01-06_70ce6e0
  - EegChannelCount=2 / EegSampleRate=250（EegMaxSampleRate=500）
  - EcgChannelCount=1 / EcgSampleRate=250（ECG 同 EEG 有 250/500 两档；DeviceInfo.EcgMaxSampleRate=250 不可靠）
  - ImpeChannelCount=3 / ImpeSampleRate=1（阻抗）
  - Acc=3 / Gyro=3 / Euler=3 / Quat=4，各 50Hz；ImuChannelCount=13（聚合 13=3+3+3+4）
  - Emg/Brth/Ppg/Spo2/Angle/Mag/Gest ChannelCount=0（不支持）

关键判定口径（与 rehab_arm 一致）：
  - setParam/getParam 流开关 key 仅 4 个有效（getParam("NTF") 单 key 探测返回 ON）：
    NTF_EEG / NTF_ECG / NTF_IMPEDANCE / NTF_IMU；其余（NTF_ACC/NTF_GYRO/...）返回
    not supported。NTF_IMU 是聚合开关，控 acc/gyro/euler/quat 四路（无独立 NTF_ACC 等 key）。
  - streams 键采用 DataType 枚举名（NTF_ACC/NTF_GYRO/NTF_EULER_DATA/NTF_QUATERNION/...），
    与实际起流观测到的 getDataType() 一致；IMU 无独立 NTF_IMU 数据类型，按四路独立投递。
  - 滤波：FILTER 聚合返回 4 个 ON（FILTER_50HZ/60HZ/HPF/LPF），单 key getParam 不可读（SDK 已知行为）。
  - 采样率：EEG 与 ECG 预期均为 250/500 两档（EegMaxSampleRate=500）。
    当前固件 V1.0.41 实测仅 250 档生效：setParam("EEG_SAMPLE_RATE","500") 报
    'unsupported sample rate 500, valid: 250'；ECG_SAMPLE_RATE key 起流前后均返回 Not supported。
    spec 按预期 250/500 记录（settable=True），500 档待固件开放后再验；IMU_SAMPLE_RATE(_LIST) 返回 50。
  - 伴随关系（co_stream）未单独起流验证，暂记 None，待专项探测确认。
"""

SPEC = {
    # ===== 身份 =====
    "model": "OB5000",
    "type": "生物电传感器（2 通道 EEG + 1 通道 ECG + 3 通道阻抗 + 6 轴 IMU）",
    "name_prefix": "OB5000",

    # ===== 数据流能力（DataType 级别，对应 DeviceInfo 的 ChannelCount 期望）=====
    # supported=True 表示该数据流应存在并产出数据；channels 为期望通道数。
    # supported=False 表示该数据流不应存在（DeviceInfo 对应 ChannelCount==0）。
    "streams": {
        "NTF_EEG":          {"supported": True,  "channels": 2},   # 脑电 2 通道，250Hz
        "NTF_ECG":          {"supported": True,  "channels": 1},   # 心电 1 通道，250/500Hz
        "NTF_IMPEDANCE":    {"supported": True,  "channels": 3},   # 阻抗 3 通道，1Hz 低频间歇
        "NTF_ACC":          {"supported": True,  "channels": 3},   # 加速度 3 轴，50Hz
        "NTF_GYRO":         {"supported": True,  "channels": 3},   # 陀螺仪 3 轴，50Hz
        "NTF_EULER_DATA":   {"supported": True,  "channels": 3},   # 欧拉角 3 轴，50Hz
        "NTF_QUATERNION":   {"supported": True,  "channels": 4},   # 四元数 4 分量，50Hz
        "NTF_IMU":          {"supported": True,  "channels": 13},  # 聚合声明 13=acc3+gyro3+euler3+quat4；实际按四路独立投递
        "NTF_EMG":          {"supported": False, "channels": 0},   # 无肌电
        "NTF_GEST":         {"supported": False, "channels": 0},   # 无手势
        "NTF_BRTH":         {"supported": False, "channels": 0},   # 无呼吸
        "NTF_PPG":          {"supported": False, "channels": 0},   # 无 PPG
        "NTF_SPO2":         {"supported": False, "channels": 0},   # 无 SpO2
        "NTF_ANGLE_DATA":   {"supported": False, "channels": 0},   # 无角度
        "NTF_MAG":          {"supported": False, "channels": 0},   # 无磁力计
    },

    # ===== NTF_IMU 聚合流的子通道组成（无独立 NTF_IMU 数据类型，按四路独立投递）=====
    "imu_sub_channels": {
        "AccChannelCount":   3,   # 加速度 3 轴
        "GyroChannelCount":  3,   # 陀螺仪 3 轴
        "EulerChannelCount": 3,   # 欧拉角 3 轴
        "QuatChannelCount":  4,   # 四元数 4 分量
    },

    # ===== 滤波开关（FILTER 聚合返回 4 个 ON）=====
    "filters": {
        "FILTER_50HZ": True,   # 50Hz 工频陷波
        "FILTER_60HZ": True,   # 60Hz 工频陷波
        "FILTER_HPF":  True,   # 高通
        "FILTER_LPF":  True,   # 低通
    },

    # ===== 采样率（统一事实来源）=====
    "sample_rates": {
        "EEG_SAMPLE_RATE": {
            # 实测（V1.0.41）setParam 500 报 'unsupported sample rate 500, valid: 250'，
            # 仅 250 生效；500 档待固件开放，按预期两档记录。
            "rates": ["250", "500"], "default": "250",
            "stream": "NTF_EEG", "settable": True, "co_stream": None,
        },
        "ECG_SAMPLE_RATE": {
            # 实测（V1.0.41）ECG_SAMPLE_RATE key 起流前后均 Not supported（SDK 未实现该参数）；
            # 按预期 250/500 两档记录，待固件开放。
            "rates": ["250", "500"], "default": "250",
            "stream": "NTF_ECG", "settable": True, "co_stream": None,
        },
        "IMU_SAMPLE_RATE": {
            "rates": ["50"], "default": "50",
            "stream": "NTF_IMU", "settable": True, "co_stream": None,
            # 无 NTF_IMU 独立数据类型：IMU 数据按四路 DataType 投递，
            # 采样率统计需聚合这四路（与 DeviceInfo.ImuChannelCount=13 一致）。
            "sub_streams": ["NTF_ACC", "NTF_GYRO", "NTF_EULER_DATA", "NTF_QUATERNION"],
        },
        "IMPEDANCE": {
            "rates": ["1"], "default": "1",
            "stream": "NTF_IMPEDANCE", "settable": False, "co_stream": None,
        },
    },
}
